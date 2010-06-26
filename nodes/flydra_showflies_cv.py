#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy

import sys
sys.path.append("/home/floris/src/floris")

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

# instructions:
# run these nodes:
# flydra_mainbrain or dummy_flydra, firefly_capture

# gui controls:
# left click: select nearest object id
# right click and drag: select 2D plane for object ids
# hit control while moving mouse, followed by shift: select distance range
# middle mouse button: clear all selections


class ImageDisplay:

    def __init__(self, device_num=0, color=True):

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
        self.field_of_view_w = np.pi*1.5 # field of view of GUI camera in radians
        self.ptf_field_of_view_w = np.pi*0.08 # field of view of the camera on the PTF system
        self.field_of_view_h = np.pi*1.5 # field of view of GUI camera in radians
        self.ptf_field_of_view_h = np.pi*0.08 # field of view of the camera on the PTF system
        
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
        # puttext puts lower left letter at (x,y) 
        
        ############################################
    
        cv.NamedWindow("Display",1)
        
        self.bridge = CvBridge()
        self.color = color

        self.dummy = True
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

        self.device_num = device_num
        sub_name = 'camera_' + str(self.device_num)
        
        rospy.Subscriber(sub_name,Image,self.image_callback)
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.flydra_callback)
        rospy.Subscriber("ptf_3d", Point, self.ptf_3d_callback)
        rospy.Subscriber("ptf_home", Point, self.ptf_home_callback)
        rospy.Subscriber("ps3_interpreter", ps3values, self.ps3_callback)
        
        self.pub_pref_obj_id = rospy.Publisher('flydra_pref_obj_id', UInt32)
        
        node_name = 'image_display' + str(self.device_num)
        rospy.init_node(node_name, anonymous=True)

        # user callbacks
        cv.SetMouseCallback("Display", self.mouse, param = None)

    def mouse(self, event, x, y, flags, param):

        # make draggable trigger rectangle
        if event == cv.CV_EVENT_RBUTTONDOWN:
            self.trigger_rectangle = [(x,y), (x,y)]
        if event == cv.CV_EVENT_MOUSEMOVE and flags == cv.CV_EVENT_FLAG_RBUTTON:
            self.trigger_rectangle[1] = (x,y)
        if event == cv.CV_EVENT_RBUTTONUP:
            self.trigger_rectangle[1] = (x,y)
            self.choose_object = False
            self.set_pref_obj_id(None)

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
    
        # left joystick: move cursor
        if ps3values.L2 > 0.99 and ps3values.R2 > 0.99:
            self.cursor = self.cursor + np.array([ps3values.joyleft_x, ps3values.joyleft_y])*self.cursorgain
            for i in range(2):
                if self.cursor[i] > 1: self.cursor[i] = 1
                if self.cursor[i] < 0: self.cursor[i] = 0
                
            # x button: mouse left click
            if ps3values.x:
                self.mouse(cv.CV_EVENT_LBUTTONUP, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), None, None)
        
            # square button: draw selection box: mouse right click and drag
            if ps3values.square is True:
                if self.square is False:
                    self.mouse(cv.CV_EVENT_RBUTTONDOWN, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), None, None)
                    self.square = True
                if self.square is True:
                    self.mouse(cv.CV_EVENT_MOUSEMOVE, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), cv.CV_EVENT_FLAG_RBUTTON, None)
            if ps3values.square is False:
                if self.square is True:
                    self.mouse(cv.CV_EVENT_RBUTTONUP, int(self.cursor[0]*self.width), int(self.cursor[1]*self.height), None, None)
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
            
    def load_parameters(self):

        # distance scale stuff:
        self.dist_scale_y = self.height-20
        self.dist_scale_x1 = 20
        self.dist_scale_x2 = self.width-20
        self.dist_scale_nticks = 10
        self.dist_scale_tick_interval = int((self.dist_scale_x2 - self.dist_scale_x1)/self.dist_scale_nticks)
        self.dist_scale_interval = (self.dist_scale_max-self.dist_scale_min)/self.dist_scale_nticks
        self.dist_scale_factor = (self.dist_scale_max - self.dist_scale_min) / (self.dist_scale_x2 - self.dist_scale_x1)
        self.dist_world_min = self.dist_scale_min
        self.dist_world_max = self.dist_scale_max
        
        self.ptf_fov_in_gui_w = int(np.sin(self.ptf_field_of_view_w/2) / np.sin(self.field_of_view_w/2) * self.width)
        self.ptf_fov_in_gui_h = int(np.sin(self.ptf_field_of_view_h/2) / np.sin(self.field_of_view_h/2) * self.height)

        self.parameters_loaded = True

    def pixel_to_dist(self, pix):
        return (pix - self.dist_scale_x1)*self.dist_scale_factor+self.dist_scale_min

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
        try:
            if self.color:
                cv_image = self.bridge.imgmsg_to_cv(data, "rgb8")
            else:
                cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
            print e

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
        
        # black background for focus bar
        cv.Rectangle(cv_image, (0,self.height-40), (self.width, self.height), self.color_black, thickness=-1)
        
        # ptf home position
        xpos = int(self.ptf_home[0]*self.width)
        ypos = int(self.ptf_home[1]*self.height)
        cv.Circle(cv_image, (xpos,ypos), 2, self.color_blue, thickness=2)
        
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
                    cx = xpos+len(str(fly.obj_id))*5
                    cy = ypos-5
                    radius = len(str(fly.obj_id))*5+5
                    cv.Circle(cv_image, (cx,cy), radius, self.color_red, thickness=1)       

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
                print self.ptf_3d
                xpos, ypos = DummyFlydra.reproject(self.ptf_3d)
                
                ptf_3d_plus_focus = [self.ptf_3d[0], self.ptf_3d[1], self.dist_world_max]
                xpf, ypf = DummyFlydra.reproject(ptf_3d_plus_focus)
                
                ptf_3d_minus_focus = [self.ptf_3d[0], self.ptf_3d[1], self.dist_world_min]
                xmf, ymf = DummyFlydra.reproject(ptf_3d_minus_focus)
            cv.Circle(cv_image, (int(xpos),int(ypos)), 2, self.color_red, thickness=2)     
            cv.Line(cv_image, (int(xmf), int(ymf)), (int(xpf), int(ypf)), self.color_red, thickness=1)
            print xmf, xpf
            
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
        cv.ShowImage("Display", cv_image)
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
    (options, args) = parser.parse_args()

    im = ImageDisplay(device_num=options.device_num)
    im.run()
