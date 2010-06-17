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

from sensor_msgs.msg import Image
from ros_flydra.msg import *
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import DummyFlydra


# instructions:
# run these nodes:
# flydra_repeater (either mainbrain, or dummy needs to be on), firefly_capture

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

        ############################################
    
        cv.NamedWindow("Display",1)
        
        self.bridge = CvBridge()
        self.color = color

        self.dummy = True
        self.flies = []
        self.obj_ids = None
        self.pref_obj_id = None
        self.trigger_rectangle = None
        self.parameters_loaded = False

        self.width = 100
        self.height = 100

        self.mousex = None
        self.mousey = None
        self.choose_object = False

        self.trigger_distance = None

        self.device_num = device_num
        sub_name = 'camera_' + str(self.device_num)
        rospy.Subscriber(sub_name,Image,self.image_callback)
        rospy.Subscriber("flydra_data", flydra_packet, self.flydra_callback)
        self.pub_pref_obj_id = rospy.Publisher('flydra_pref_obj_id', String)
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

        self.parameters_loaded = True

    def pixel_to_dist(self, pix):
        return (pix - self.dist_scale_x1)*self.dist_scale_factor+self.dist_scale_min

    def clear(self):
        self.trigger_rectangle = None
        self.set_pref_obj_id(None)
        self.trigger_distance = None
        self.load_parameters()

    def flydra_callback(self, data):
        self.flies = data.flies        
        self.obj_ids = [fly.obj_id for fly in self.flies]
        if self.pref_obj_id not in self.obj_ids and self.pref_obj_id is not None:
            self.set_pref_obj_id(None)
            print 'lost the preferred fly! please select a new one... (mouseclick)'
            
    def set_pref_obj_id(self, obj_id):
        self.pref_obj_id = obj_id
        self.pub_pref_obj_id.publish(String(self.pref_obj_id))
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
                    if index+1 > len(self.flies)-1:
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

        # text fonts
        font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.5,0.5)
        small_font = cv.InitFont(cv.CV_FONT_HERSHEY_TRIPLEX,0.4,0.4)
        color_green = cv.Scalar(0,255,0,0)
        color_light_green = cv.Scalar(175,255,175,0)
        color_red = cv.Scalar(0,0,255,0)
        color_white = cv.Scalar(255,255,255,0)
        # puttext puts lower left letter at (x,y) 

        # print some basic (static) information
        existing_objs = 'existing objects: ' + str(self.obj_ids)
        cv.PutText(cv_image, existing_objs, (0, 15), font, color_green)
        pref_obj_id = 'pref obj id: ' + str(self.pref_obj_id)
        cv.PutText(cv_image, pref_obj_id, (0, 35), font, color_green)

        # draw the flies    
        if len(self.flies) is not None:
            if self.choose_object:
                opt_dist = 1000 # initialization for finding nearest fly
                choose_fly =  self.obj_ids[0] # initialization for nearest fly
            for fly in self.flies:
                # reproject fly onto camera... 
                if self.dummy:
                    xpos, ypos = DummyFlydra.reproject(fly.state_vecs[0:3])
                    dist = linalg.norm(fly.state_vecs[0:3])

                # if xpos, ypos inside rectangle set pref obj id
                if self.trigger_rectangle is not None and self.pref_obj_id is None:
                    if xpos >= self.trigger_rectangle[0][0] and xpos <= self.trigger_rectangle[1][0]:
                        if ypos >= self.trigger_rectangle[0][1] and ypos <= self.trigger_rectangle[1][1]:
                            if dist <= self.dist_world_max and dist >= self.dist_world_min:
                                self.set_pref_obj_id(fly.obj_id)

                # if objects out of range, use light green
                if dist <= self.dist_world_max and dist >= self.dist_world_min:
                    # obj_id text
                    cv.PutText(cv_image, str(fly.obj_id), (xpos, ypos), font, color_green) 
                    # distance text
                    cv.PutText(cv_image, str(dist)[0:4], (xpos, ypos+10), small_font, color_green)
                else:
                    # obj_id text
                    cv.PutText(cv_image, str(fly.obj_id), (xpos, ypos), font, color_light_green) 
                    # distance text
                    cv.PutText(cv_image, str(dist)[0:4], (xpos, ypos+10), small_font, color_light_green)
                
                if fly.obj_id == self.pref_obj_id:
                    cx = xpos+len(str(fly.obj_id))*5
                    cy = ypos-5
                    radius = len(str(fly.obj_id))*5+5
                    cv.Circle(cv_image, (cx,cy), radius, color_red, thickness=1)       

                # choose nearest object to last mouse click 
                if self.choose_object:
                    dist = np.abs(xpos-self.mousex)+np.abs(ypos-self.mousey)
                    if dist < opt_dist:
                        opt_dist = dist
                        choose_fly = fly.obj_id  

            if self.choose_object:
                self.set_pref_obj_id(choose_fly)
                self.choose_object = False              
        

        # draw the rectangle:
        if self.trigger_rectangle is not None:
            cv.Rectangle(cv_image, self.trigger_rectangle[0], self.trigger_rectangle[1], color_red, thickness=1)

        # distance scale:
        cv.Line(cv_image, (self.dist_scale_x1,self.dist_scale_y), (self.dist_scale_x2,self.dist_scale_y), color_white, thickness=2)
        dist_str = 'dist from cam, ' + self.dist_scale_units
        cv.PutText(cv_image, dist_str, (self.dist_scale_x1,self.dist_scale_y+15), small_font, color_white)
        for n in range(self.dist_scale_nticks+1):
            y1 = self.dist_scale_y+5
            y2 = self.dist_scale_y-5
            x = self.dist_scale_x1+self.dist_scale_tick_interval*n
            dist = self.dist_scale_min+self.dist_scale_interval*n
            cv.Line(cv_image, (x,y1), (x,y2), color_white, thickness=2)
            cv.PutText(cv_image, str(dist)[0:4], (x,y2), small_font, color_white)

        # trigger distance rectangle:
        if self.trigger_distance is not None:
            cv.Rectangle(cv_image, self.trigger_distance[0], self.trigger_distance[1], color_red, thickness=1)
            

        

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

    im = ImageDisplay()
    im.run()
