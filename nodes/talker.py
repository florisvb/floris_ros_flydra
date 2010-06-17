#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time

def talker():
    pub = rospy.Publisher('chatter', Float64Arr)
    rospy.init_node('talker')
    start_time = time.time()
    while not rospy.is_shutdown():
        msg = [start_time, time.time()-start_time]
        rospy.loginfo(msg)
        pub.publish(Float64Arr(msg))
        rospy.sleep(1.0)
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

