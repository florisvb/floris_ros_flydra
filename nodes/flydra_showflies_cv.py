#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy

import sys
sys.path.append("/home/floris/src/floris")
sys.path.append("/home/floris/src/flydra/flydra")
sys.path.append("/home/floris/ros/stacks/ros_hydra/nodes")

import time
import numpy as np
import scipy.linalg as linalg
import cv
import motmot.cam_iface.cam_iface_ctypes as cam_iface
import cvNumpy
from optparse import OptionParser

from sensor_msgs.msg import Image
from ros_flydra.msg import *
from std_msgs.msg import *
from joystick_ps3.msg import ps3values
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError

import DummyFlydra
import reconstruct

import pan_tilt_focus

# instructions:
# run these nodes:
# flydra_mainbrain or dummy_flydra, firefly_capture

# gui controls:
# left click: select nearest object id
# right click and drag: select 2D plane for object ids
# hit control while moving mouse, followed by shift: select distance range
# middle mouse button: clear all selections

## TO DO ##
# currently in fly position the distance is the distance to flydra - not the ptf camera

# field of view calculator:
def FOV(frame_size, focal_length):
    fov =  2 * np.arctan2 (frame_size,(focal_length * 2))
    return fov
    
def sensor_dims(diagonal, res_w, res_h):
    theta = np.arctan2(float(res_h),float(res_w))
    size_w = np.cos(theta)*diagonal
    size_h = np.sin(theta)*diagonal
    return size_w, size_h
    
class ImageDisplay:

    def __init__(self,  calibration=None,
                        camid=None,
                        device_num=0, 
                        color=True, 
                        calibration_filename=None, 
                        dummy=True,
                        gui_focal_length=6,
                        gui_sensor_w=5,
                        gui_sensor_h=3,
                        ptf_focal_length=400,
                        ptf_sensor_w=20,
                        ptf_sensor_h=15):

        ## values that should be function inputs ##
        
        # distance parameters:
        self.dist_scale_min = 0.
        self.dist_scale_max = 20.
        self.dist_scale_units = 'm'
        #
        
        # ps3 controller parameters:
        self.cursorgain = 0.03
        self.ptf_home = [0,0,0]
        
        # camera parameters - field of view, width and height:
        
        # FOV (rectilinear) =  2 * arctan (frame size/(focal length * 2))
        self.field_of_view_w = FOV(gui_sensor_w,gui_focal_length) # field of view of GUI camera in radians
        self.ptf_field_of_view_w = FOV(ptf_sensor_w,ptf_focal_length) # field of view of the camera on the PTF system
        self.field_of_view_h = FOV(gui_sensor_h,gui_focal_length) # field of view of GUI camera in radians
        self.ptf_field_of_view_h = FOV(ptf_sensor_h,ptf_focal_length) # field of view of the camera on the PTF system
        self.camera_center = [0,0,0]
        
        # camera calibration:
        self.dummy = dummy
        self.cam_id = camid
        if self.dummy is not None:
            if calibration_filename is not None:
                self.camera_calibration = reconstruct.Reconstructor(calibration_filename)
                self.cam_id = self.camera_calibration.get_cam_ids()[0]
            if calibration is not None:
                self.camera_calibration = reconstruct.Reconstructor(calibration)
            else:
                print 'WARNING: running with a dummy calibration, please enter a calibration_filename'
                self.dummy = True
        
        # display parameters:
        # text fonts
        self.font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        self.small_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.4,0.4)
        self.color_green = cv.Scalar(0,255,0,0)
        self.color_light_green = cv.Scalar(175,255,175,0)
        self.color_red = cv.Scalar(0,0,255,0)
        self.color_blue = cv.Scalar(255,0,0,0)
        self.color_purple = cv.Scalar(255,0,255,0)
        self.color_white = cv.Scalar(255,255,255,0)
        self.color_black = cv.Scalar(0,0,0,0)
        self.active = 1
        # puttext puts lower left letter at (x,y) 
        
        ############################################
        self.display_name = "Display Cam: " + str(self.cam_id)
        cv.NamedWindow(self.display_name,1)
        
        self.bridge = CvBridge()
        self.color = color

        self.objects = []
        self.obj_ids = None
        self.pref_obj_id = None
        self.trigger_rectangle = None
        self.parameters_loaded = False

        self.width = 100
        self.height = 100
        
        self.cursor = np.array([0.5,0.5]) # keep between 0 and 1
        self.square = False
        self.circle = False

        self.mousex = None
        self.mousey = None
        self.motor_pos_2d = None
        self.choose_object = False
        self.ptf_3d = None

        self.trigger_distance = None

        node_name = 'image_display' + str(self.cam_id)
        rospy.init_node(node_name, anonymous=True)
        print 'node initialized' 
        
        
        try: 
            topic = self.cam_id + '/image_raw'
        except:
            raise ValueError('please enter a valid cam_id')
        rospy.Subscriber(topic,Image,self.image_callback)

        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.flydra_callback)
        rospy.Subscriber("ptf_3d", Point, self.ptf_3d_callback)
        rospy.Subscriber("ptf_home", Point, self.ptf_home_callback)
        rospy.Subscriber("ps3_interpreter", ps3values, self.ps3_callback)
        print 'subscribing to ps3_interpreter'
        rospy.Subscriber("camera_center", Point, self.camera_center_callback)
        
        self.pub_pref_obj_id = rospy.Publisher('flydra_pref_obj_id', UInt32)
        
        # user callbacks
        cv.SetMouseCallback(self.display_name, self.mouse, param = None)

    def mouse(self, event, x, y, flags, param):
    
        # make draggable trigger rectangle
        if event == cv.CV_EVENT_RBUTTONUP:
            self.active *= -1

        # clear selections
        if event == cv.CV_EVENT_MBUTTONUP:
            print 'cleared selections!'
            self.clear()

        # choose obj id
        if event == cv.CV_EVENT_LBUTTONUP:
            self.mousex = x
            self.mousey = y
            self.choose_pref_obj_id()
                            
        # distance trigger rectangle
        if flags == cv.CV_EVENT_FLAG_CTRLKEY:
            self.trigger_distance = [(x,self.dist_scale_y-5), (x,self.dist_scale_y+5)]
            self.dist_world_min = self.pixel_to_dist(x)
        if flags == cv.CV_EVENT_FLAG_SHIFTKEY:
            self.trigger_distance[1] = (x,self.dist_scale_y+5)
            self.dist_world_max = self.pixel_to_dist(x)
            
    def ps3_callback(self, ps3values):
    
        # if experiencing joystick errors - hit L2 and R2 before using controller - values initialize at 0 for some strange reason
    
        # only on active window!
        if self.active == 1:
        
            x = int(self.cursor[0]*self.width)
            y = int(self.cursor[1]*self.height)
    
            # left joystick: move cursor
            if ps3values.L2 > 0.99 and ps3values.R2 > 0.99:
                self.cursor = self.cursor + np.array([ps3values.joyleft_x, ps3values.joyleft_y])*self.cursorgain
                for i in range(2):
                    if self.cursor[i] > 1: self.cursor[i] = 1
                    if self.cursor[i] < 0: self.cursor[i] = 0
                    
                # x button: mouse left click
                if ps3values.x:
                    self.mouse(cv.CV_EVENT_LBUTTONUP, x, y, None, None)
            
                # square button: draw selection box: mouse right click and drag
                if ps3values.square is True:
                    if self.square is False:
                        self.trigger_rectangle = [(x,y), (x,y)]
                        self.square = True
                    if self.square is True:
                        self.trigger_rectangle[1] = (x,y)
                if ps3values.square is False:
                    if self.square is True:
                        self.choose_object = False
                        self.square = False
                        
                # start button: clear selections: mouse middle button up
                if ps3values.start is True:
                    self.mouse(cv.CV_EVENT_MBUTTONUP, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), None, None)
                    
                    
                # circle button: distance trigger rectangle
                if ps3values.circle is True:
                    if self.circle is False:
                        self.mouse(cv.CV_EVENT_MOUSEMOVE, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), cv.CV_EVENT_FLAG_CTRLKEY, None)
                        self.circle = True
                    if self.circle is True:
                        self.mouse(cv.CV_EVENT_MOUSEMOVE, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), cv.CV_EVENT_FLAG_SHIFTKEY, None)
                if ps3values.circle is False:
                    if self.circle is True:
                        self.mouse(cv.CV_EVENT_MOUSEMOVE, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), cv.CV_EVENT_FLAG_SHIFTKEY, None)
                        self.circle = False
            
            
    def ptf_3d_callback(self, data):
        self.ptf_3d = [data.x, data.y, data.z]       
        
    def ptf_home_callback(self,data):
         self.ptf_home = [data.x, data.y, data.z]      
         
    def camera_center_callback(self, data):
        self.camera_center = [data.x, data.y, data.z]      
            
    def load_parameters(self):

        # distance scale stuff:
        self.dist_scale_y = self.height-20
        self.dist_scale_x1 = 20
        self.dist_scale_x2 = self.width-20
        self.dist_scale_nticks = 10
        self.dist_scale_tick_interval = int((self.dist_scale_x2 - self.dist_scale_x1)/self.dist_scale_nticks)
        self.dist_scale_interval = (self.dist_scale_max-self.dist_scale_min)/self.dist_scale_nticks
        self.dist_scale_factor = (self.dist_scale_max - self.dist_scale_min) / (self.dist_scale_x2 - self.dist_scale_x1) # meters/pixel
        self.dist_world_min = self.dist_scale_min
        self.dist_world_max = self.dist_scale_max
        
        self.ptf_fov_in_gui_w = int(np.sin(self.ptf_field_of_view_w/2) / np.sin(self.field_of_view_w/2) * self.width)
        self.ptf_fov_in_gui_h = int(np.sin(self.ptf_field_of_view_h/2) / np.sin(self.field_of_view_h/2) * self.height)

        self.parameters_loaded = True        
        
    def dist_to_pixel(self,dist):
        #print 'dist to pixel: '
        #print dist, self.dist_scale_min, self.dist_scale_factor, self.dist_scale_x1
        return ( (dist-self.dist_scale_min)/self.dist_scale_factor + self.dist_scale_x1)

    def pixel_to_dist(self, pix):
        dist = (pix - self.dist_scale_x1)*self.dist_scale_factor+self.dist_scale_min
        #print dist
        return dist

    def clear(self):
        self.trigger_rectangle = None
        self.set_pref_obj_id(None)
        self.trigger_distance = None
        self.load_parameters()

    def flydra_callback(self, super_packet):
        for packet in super_packet.packets:
            self.objects = packet.objects        
        self.obj_ids = [obj.obj_id for obj in self.objects]
        if self.pref_obj_id not in self.obj_ids and self.pref_obj_id is not None:
            self.set_pref_obj_id(None)
            print 'lost the preferred fly! please select a new one... (mouseclick)'
            
    def set_pref_obj_id(self, obj_id):
        self.pref_obj_id = obj_id
        self.pub_pref_obj_id.publish(UInt32(self.pref_obj_id))
        print 'new pref obj id! ', obj_id
        return

    def choose_pref_obj_id(self, mode='nearest'):
        # choose a new object ID!

        # two possibilities: 
        # 1. we have not yet selected a obj id
        # 2. we want to change the current obj id
        
        if mode is 'cycle':
            try:  
                if self.pref_obj_id is not None:
                    index = self.obj_ids.index(self.pref_obj_id)
                    new_index = index+1
                    if index+1 > len(self.objects)-1:
                        new_index = 0
                    self.set_pref_obj_id(self.obj_ids[new_index])
                    print 'chose a new pref_obj_id: ', self.pref_obj_id
                    return

                if self.pref_obj_id is None:
                    # choose the oldest obj_id
                    self.set_pref_obj_id(self.obj_ids[0])
                    print 'chose a new pref_obj_id: ', self.pref_obj_id
                    return
            except: 
                print 'no flies to choose from!'
                return

        if mode is 'nearest':
            self.choose_object = True
            # choose next time we're in the draw frame (so we don't recalculate fly pos constantly)
            return
            
    def image_callback(self,data):
        encoding = data.encoding
        img = self.bridge.imgmsg_to_cv(data, desired_encoding="passthrough")
        cv_image = cv.CreateImage(cv.GetSize(img),cv.IPL_DEPTH_8U,3)
        
        if data.encoding == 'bayer_bggr8':
            cv.CvtColor(img, cv_image,cv.CV_BayerRG2RGB)
        else:
            print 'warning: bayer type not recognized, using monochrome image instead'
            cv_image = img        
                
        if self.parameters_loaded is False:
            self.width = cv.GetSize(cv_image)[0]
            self.height = cv.GetSize(cv_image)[1]
            self.load_parameters()

        ##########################################################
        # Drawing Functions go here, operate on cv_image

        # print some basic (static) information
        existing_objs = 'existing objects: ' + str(self.obj_ids)
        cv.PutText(cv_image, existing_objs, (0, 15), self.font, self.color_green)
        pref_obj_id = 'pref obj id: ' + str(self.pref_obj_id)
        cv.PutText(cv_image, pref_obj_id, (0, 35), self.font, self.color_green)
        
        if self.active == 1:
            cv.PutText(cv_image, 'active', (self.width-100, 15), self.font, self.color_red)
        elif self.active == -1:
            cv.PutText(cv_image, 'dormant', (self.width-100, 15), self.font, self.color_green)
            
        # black background for focus bar
        cv.Rectangle(cv_image, (0,self.height-40), (self.width, self.height), self.color_black, thickness=-1)
        
        # draw the flies    
        if len(self.objects) > 0:
            if self.choose_object:
                opt_dist = 1000 # initialization for finding nearest fly
                choose_fly =  self.obj_ids[0] # initialization for nearest fly
            for fly in self.objects:
                # reproject fly onto camera... 
                if self.dummy:
                    pos = [fly.position.x, fly.position.y, fly.position.z]
                    xpos, ypos = DummyFlydra.reproject(pos)
                    dist = linalg.norm(pos)
                if not self.dummy:
                    pos = [fly.position.x, fly.position.y, fly.position.z]
                    xpos, ypos = self.camera_calibration.find2d(self.cam_id, pos, distorted=True)
                    xpos = int(xpos)
                    ypos = int(ypos)
                    cv.Circle(cv_image, (xpos,ypos), 1, self.color_red, thickness=1)    
                    dist = linalg.norm(np.array(pos)-np.array(self.camera_center))

                # if xpos, ypos inside rectangle set pref obj id
                if self.trigger_rectangle is not None and self.pref_obj_id is None:
                    if xpos >= self.trigger_rectangle[0][0] and xpos <= self.trigger_rectangle[1][0]:
                        if ypos >= self.trigger_rectangle[0][1] and ypos <= self.trigger_rectangle[1][1]:
                            if dist <= self.dist_world_max and dist >= self.dist_world_min:
                                self.set_pref_obj_id(fly.obj_id)

                # if objects out of range, use light green
                if dist <= self.dist_world_max and dist >= self.dist_world_min:
                    # obj_id text
                    cv.PutText(cv_image, str(fly.obj_id), (xpos, ypos), self.font, self.color_green) 
                    # distance text
                    cv.PutText(cv_image, str(dist)[0:4], (xpos, ypos+10), self.small_font, self.color_green)
                else:
                    # obj_id text
                    cv.PutText(cv_image, str(fly.obj_id), (xpos, ypos), self.font, self.color_light_green) 
                    # distance text
                    cv.PutText(cv_image, str(dist)[0:4], (xpos, ypos+10), self.small_font, self.color_light_green)
                
                if fly.obj_id == self.pref_obj_id:
                    cv.PutText(cv_image, str(fly.obj_id), (xpos, ypos), self.font, self.color_red) 
                    # circle for covariance
                    cov = np.sum(np.abs(fly.posvel_covariance_diagonal[0:3]))
                    cx = xpos+len(str(fly.obj_id))*5
                    cy = ypos-5
                    #radius = len(str(fly.obj_id))*5+5
                    cv.Circle(cv_image, (cx,cy), cov*10, self.color_red, thickness=1)    
                    cv.PutText(cv_image, str(cov), (xpos, ypos+20), self.small_font, self.color_red)   

                # choose nearest object to last mouse click 
                if self.choose_object:
                    dist = np.abs(xpos-self.mousex)+np.abs(ypos-self.mousey)
                    if dist < opt_dist:
                        opt_dist = dist
                        choose_fly = fly.obj_id  

            if self.choose_object:
                self.set_pref_obj_id(choose_fly)
                self.choose_object = False              
        

        # trigger rectangle:
        if self.trigger_rectangle is not None:
            cv.Rectangle(cv_image, self.trigger_rectangle[0], self.trigger_rectangle[1], self.color_blue, thickness=1)

        # distance scale:
        cv.Line(cv_image, (self.dist_scale_x1,self.dist_scale_y), (self.dist_scale_x2,self.dist_scale_y), self.color_white, thickness=2)
        dist_str = 'dist from cam, ' + self.dist_scale_units
        cv.PutText(cv_image, dist_str, (self.dist_scale_x1,self.dist_scale_y+15), self.small_font, self.color_white)
        for n in range(self.dist_scale_nticks+1):
            y1 = self.dist_scale_y+5
            y2 = self.dist_scale_y-5
            x = self.dist_scale_x1+self.dist_scale_tick_interval*n
            dist = self.dist_scale_min+self.dist_scale_interval*n
            cv.Line(cv_image, (x,y1), (x,y2), self.color_white, thickness=2)
            cv.PutText(cv_image, str(dist)[0:4], (x,y2), self.small_font, self.color_white)

        # trigger distance rectangle:
        if self.trigger_distance is not None:
            cv.Rectangle(cv_image, self.trigger_distance[0], self.trigger_distance[1], self.color_blue, thickness=1)
            
        # PTF position
        if self.ptf_3d is not None:
        
            if self.dummy:
                xpos, ypos = DummyFlydra.reproject(self.ptf_3d)
                xhome = 0
                yhome = 0
            if not self.dummy and self.ptf_3d is not None:
                xpos, ypos = self.camera_calibration.find2d(self.cam_id, self.ptf_3d, distorted=True)
                xhome, yhome = self.camera_calibration.find2d(self.cam_id, self.ptf_home, distorted=True)
            #print '*'*80
            #print self.ptf_3d, self.camera_center
            dist = linalg.norm( np.array(self.ptf_3d) - np.array(self.camera_center))
            home_dist = linalg.norm( np.array(self.ptf_home) - np.array(self.camera_center))
            pix = self.dist_to_pixel(dist)
            home_pix = self.dist_to_pixel(home_dist)
            #print '**', pix, self.dist_scale_y
            cv.Circle(cv_image, (int(pix),int(self.dist_scale_y)), 2, self.color_red, thickness=2)   
            cv.Circle(cv_image, (int(home_pix),int(self.dist_scale_y)), 2, self.color_blue, thickness=2)   
            cv.Circle(cv_image, (int(xpos),int(ypos)), 2, self.color_red, thickness=2)    
            cv.Circle(cv_image, (int(xhome),int(yhome)), 2, self.color_blue, thickness=2)      

            #print 'ptf_3d: ', self.ptf_3d, 'xpos, ypos: ', xpos, ypos
            
            UL = (xpos-self.ptf_fov_in_gui_w, ypos-self.ptf_fov_in_gui_h)
            LR = (xpos+self.ptf_fov_in_gui_w, ypos+self.ptf_fov_in_gui_h)
            cv.Rectangle(cv_image, UL, LR, self.color_red, thickness=1)

        # draw cursor:
        xpos = int(self.cursor[0]*self.width)
        ypos = int(self.cursor[1]*self.height)
        cv.Circle(cv_image, (xpos,ypos), 2, self.color_purple, thickness=2)
        cv.Line(cv_image, (xpos-10, ypos), (xpos+10,ypos), self.color_purple, thickness=1)
        cv.Line(cv_image, (xpos, ypos-10), (xpos,ypos+10), self.color_purple, thickness=1)

        ##########################################################
        cv.ShowImage(self.display_name, cv_image)
        cv.WaitKey(3)

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down"
        cv.DestroyAllWindows()

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--device-num", type="int", dest="device_num", default=0,
                        help="camera device number")
    parser.add_option("--calibration-filename", type="string", dest="calibration_filename", default=None,
                        help="camera calibration filename, currently needs to be a single camera calibration")
    parser.add_option("--dummy", action='store_true', dest="dummy", default=False,
                        help="run using a dummy calibration, for testing")
    parser.add_option("--gui-focal-length", type="float", dest="gui_focal_length", default=6,
                        help="focal length of the lens used on the camera for the gui system (mm)")
    parser.add_option("--gui-sensor-type", type="string", dest="gui_sensor_type", default='1/3',
                        help="sensor type for the gui camera, ie. '1/3', '1/2' etc.")
    parser.add_option("--ptf-focal-length", type="float", dest="ptf_focal_length", default=400,
                        help="focal length of the lens used on the ptf camera (mm)")
    parser.add_option("--ptf-sensor-type", type="string", dest="ptf_sensor_type", default='APSC-1.6x',
                        help="sensor type for the ptf camera, ie. 'APSC-1.6x', 'fullframe' etc.")
                        
    parser.add_option("--camid", type="string", dest="camid", default='None',
                        help='camid of the desired camnode camera to use')
    parser.add_option("--calibration", type="string", dest="calibration", default=None,
                        help="flydra camera calibration filename")
    
    (options, args) = parser.parse_args()
    
    if options.calibration_filename is not None:
        options.dummy = False
        
    if options.gui_sensor_type == '1/3':
        gui_sensor_w, gui_sensor_h = sensor_dims(8.4666, 752, 480)

    if options.ptf_sensor_type == 'APSC-1.6x':
        ptf_sensor_w, ptf_sensor_h = sensor_dims(35./1.6, 3, 2)
    if options.ptf_sensor_type == 'fullframe':
        ptf_sensor_w, ptf_sensor_h = sensor_dims(35., 3, 2)

    im = ImageDisplay(  calibration=options.calibration,
                        camid=options.camid,
                        device_num=options.device_num, 
                        dummy=options.dummy, 
                        calibration_filename=options.calibration_filename, 
                        gui_focal_length=options.gui_focal_length,
                        gui_sensor_w=gui_sensor_w,
                        gui_sensor_h=gui_sensor_h,
                        ptf_focal_length=options.ptf_focal_length,
                        ptf_sensor_w=ptf_sensor_w,
                        ptf_sensor_h=ptf_sensor_h)
    im.run()
    
    
    
    
    
    
    
    
    
    
