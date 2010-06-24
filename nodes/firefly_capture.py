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

from optparse import OptionParser

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# this node talks to cameras using Andrew Straw's motmot.cam_iface
# works with pointgrey fireflies (usb)
# returns either color or RGB images, sent over Image msgs

class Camera:
    def __init__(self, mode_num=4, framerate=30, device_num=0, num_buffers=32, color=True, bayer='RG'):
                            
        # initialize firefly camera using Andrew Straw's motmot.cam_iface
        self.mode_num = mode_num
        self.device_num = device_num
        self.num_buffers = num_buffers
        self.cam = cam_iface.Camera(self.device_num,self.num_buffers,self.mode_num)
        self.cam.start_camera()
        time.sleep(1) # to help reduce corrupt frames
        self.cam.set_framerate(framerate)
        self.fps = self.cam.get_framerate()
        print 'framerate: ', self.fps
        self.color = color
        self.bayer = bayer # choices are RG, GR, BG, GB

        self.images_initialized = False
    
        # ROS
        pub_name = 'camera_' + str(self.device_num)
        self.bridge = CvBridge()
        self.pub_img = rospy.Publisher(pub_name, Image)
        rospy.init_node(pub_name)

    def initialize_color_images(self):
        print cv.GetSize(self.frame)
        self.image = cv.CreateImage(cv.GetSize(self.frame),cv.IPL_DEPTH_8U,3)
        self.images_initialized = True

    def grab_next_frame(self):
        buf_array = None
        while buf_array is None:
            try: 
                buf_array = np.asarray(self.cam.grab_next_frame_blocking())
                #print 'got an image'
            except:
                print 'corrupt frame...'
                continue

        # buf_array is the numpy array
        ################################
        # image processing on buf_array would go here, 
        # hack to send mono array of numpy data:
        # msg = CvBridge().cv_to_imgmsg(cv.fromarray(a))
        # a = numpy.fromarray(CvBridge().imgmsg_to_cv(msg))
        ################################

        # now publish image, color or mono
        self.frame = cvNumpy.array_to_im(buf_array)
        if self.color:
            if self.images_initialized is False:
                self.initialize_color_images()
            if self.bayer == 'GB':
                cv.CvtColor(self.frame, self.image,cv.CV_BayerGB2RGB)
            if self.bayer == 'BG':
                cv.CvtColor(self.frame, self.image,cv.CV_BayerBG2RGB)
            if self.bayer == 'GR':
                cv.CvtColor(self.frame, self.image,cv.CV_BayerGR2RGB)
            if self.bayer == 'RG':
                cv.CvtColor(self.frame, self.image,cv.CV_BayerRG2RGB)
            self.pub_img.publish(self.bridge.cv_to_imgmsg(self.image, "rgb8"))
        else:
            self.image = self.frame
            self.pub_img.publish(self.bridge.cv_to_imgmsg(self.image, "mono8"))

    def run(self):
        while not rospy.is_shutdown():
            self.grab_next_frame()

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--device-num", type="int", dest="device_num", default=0,
                        help="camera device number")
    parser.add_option("--mode-num", type="int", dest="mode_num", default=4, 
                        help=
                        '''Camera mode number:
                        0: 752x240, bayer=BG (closest)
                        1: 1504x240, not a great mode
                        2: 376x240, no color (bayer irrelevant)
                        3: 752x240, not a great mode
                        4: Normal (752x480), bayer=RG''')
    parser.add_option("--framerate", type="int", dest="framerate", default=30)
    parser.add_option("--num-buffers", type="int", dest="num_buffers", default=32)
    parser.add_option("--color", action="store_true", dest="color", default=True)
    parser.add_option("--mono", action="store_false", dest="color")
    parser.add_option("--bayer", type="string", dest="bayer", default='RG')
    (options, args) = parser.parse_args()

    camera = Camera(mode_num=options.mode_num, 
                    framerate=options.framerate, 
                    device_num=options.device_num, 
                    num_buffers=options.num_buffers, 
                    color=options.color,
                    bayer=options.bayer)

    camera.run()


