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
            print
            print '*'*80
            print now-packet.acquire_stamp.to_seconds()
            for obj in packet.objects:
                position = [obj.position.x, obj.position.y, obj.position.z]
                velocity = [obj.velocity.x, obj.velocity.y, obj.velocity.z]
                print
                print position
                print velocity
            
        #self.packets = super_packet.packets
    

if __name__ == '__main__':
    listener = Listener()

