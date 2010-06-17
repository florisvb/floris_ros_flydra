#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time

def callback(data):
    msg_sent = data.data[0]+data.data[1]
    msg_recieved = time.time()
    print 'latency: ', msg_recieved-msg_sent

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("chatter", Float64Arr, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

