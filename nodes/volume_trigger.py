#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time
from std_msgs.msg import *
import socket
import common_variables

class SA1:
    def __init__(self):
        self.hostname = '127.0.0.1'
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class Listener:
    def __init__(self):
    
        # trigger parameters:
        self.z_max = 1
        self.z_min = 0
        self.r_max = 0.03
        self.refractory_time = 10
        self.last_time = time.time()

        # SA1 intialization
        self.SA1 = SA1()
        
        # ROS
        self.pub = rospy.Publisher('volume_trigger', Bool)
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.callback)
        rospy.init_node('listener', anonymous=True)
        rospy.spin()

    def callback(self, super_packet):
        for packet in super_packet.packets:
            for obj in packet.objects:
                position = [obj.position.x, obj.position.y, obj.position.z]
                self.check_volume(position)
                
    def check_volume(self, pos3d):
    
        x = pos3d[0]
        y = pos3d[1]
        z = pos3d[2]

        send_trigger = 0        
        current_time = time.time()
            
        # for a cylinder:
        if z < self.z_max and z > self.z_min:
            r = np.sqrt( x**2 + y**2 )
            if r < self.r_max:
                send_trigger = 1
            
        # check refractory time:
        if send_trigger == 1:
            if current_time - self.last_time < self.refractory_time:
                send_trigger = 0
                
        if send_trigger == 1:
            self.pub.publish(1)
            self.trigger_SA1()
            self.last_time = time.time()
        else:
            self.pub.pubish(0)
            
    def trigger_SA1(self):
        self.SA1.sender.sendto('3',(self.SA1.hostname,common_variables.trigger_network_socket_port))
        print 'triggered!, time: ', self.last_time

if __name__ == '__main__':
    listener = Listener()

