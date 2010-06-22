#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time


class Listener:
    def __init__(self):
        
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.callback)
        rospy.init_node('listener', anonymous=True)
        rospy.spin()

    def callback(self, super_packet):
        print 'super packet, ', len(super_packet.packets), ' packets'
        now = time.time()
        for packet in super_packet.packets:
            print now-packet.acquire_stamp.to_seconds()
            print now-packet.reconstruction_stamp.to_seconds()
            
        #self.packets = super_packet.packets
    

if __name__ == '__main__':
    listener = Listener()

