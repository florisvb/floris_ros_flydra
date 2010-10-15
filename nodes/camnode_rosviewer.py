#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy

import time
import numpy as np

import cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from optparse import OptionParser

class ImageDisplay:

    def __init__(self, cam_id=None, topic=None):
        self.bridge = CvBridge()
        
        if topic is not None:
            self.cam_id = topic.partition('/')[0]
        else:
            try: 
                self.cam_id = cam_id
                topic = cam_id + '/image_raw'
            except:
                raise ValueError('please enter a valid cam_id or rostopic name')
                
        self.display_name = self.cam_id + '_display'
        cv.NamedWindow(self.display_name,1)
        self.image_sub = rospy.Subscriber(topic,Image,self.image_callback)
        rospy.init_node(self.display_name, anonymous=True)
        
        self.lasttime = time.time()
        self.run()

    def image_callback(self,data):
        now = time.time()
        print 1./(now-self.lasttime)
        cv_image = self.bridge.imgmsg_to_cv(data, data.encoding)
        cv.ShowImage(self.display_name, cv_image)
        cv.WaitKey(3)
        self.lasttime = now
    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print "Shutting down"
        cv.DestroyAllWindows()

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--topic", type="str", dest="topic", default=None,
                        help="camnode topic ID: ie. for flydra_cameranode: 'cam_id/raw_image'")
    parser.add_option("--camid", type="str", dest="cam_id", default=None,
                        help="camid for camnode topic")
    (options, args) = parser.parse_args()

    im = ImageDisplay(  cam_id=options.cam_id, topic=options.topic)
    im.run()
