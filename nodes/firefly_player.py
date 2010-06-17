#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy

import sys
sys.path.append("/home/floris/src/floris")

import time
import numpy as np
import cv
import motmot.cam_iface.cam_iface_ctypes as cam_iface
import cvNumpy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from optparse import OptionParser

class ImageDisplay:

    def __init__(self, device_num=0, color=True):
        cv.NamedWindow("Display",1)
        
        self.bridge = CvBridge()
        self.color = color

        self.device_num = device_num
        sub_name = 'camera_' + str(self.device_num)
        self.image_sub = rospy.Subscriber(sub_name,Image,self.image_callback)
        node_name = 'image_display' + str(self.device_num)
        rospy.init_node(node_name, anonymous=True)

    def image_callback(self,data):
        try:
            if self.color:
                cv_image = self.bridge.imgmsg_to_cv(data, "rgb8")
            else:
                cv_image = self.bridge.imgmsg_to_cv(data, "mono8")
        except CvBridgeError, e:
          print e

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
    parser.add_option("--color", action="store_true", dest="color", default=True)
    parser.add_option("--mono", action="store_false", dest="color")
    (options, args) = parser.parse_args()



    im = ImageDisplay(  device_num=options.device_num, 
                        color=options.color)
    im.run()
