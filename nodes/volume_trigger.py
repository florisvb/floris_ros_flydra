#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time
from std_msgs.msg import *
import socket
import numpy as np

import sys
sys.path.append('/home/floris/src/flydra/flydra')
import common_variables

class SA1:
    def __init__(self):
        self.hostname = '127.0.0.1'
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    def trigger_SA1(self, obj):
        print_time=False
        self.sender.sendto('3',(self.hostname,common_variables.trigger_network_socket_port))
        if print_time:
            print 'triggered!, time: ', time.time()
            
def example_trigger_function(obj):
    print
    print [obj.position.x, obj.position.y, obj.position.z]
    print 'triggered!' 
    print
        
class VolumeTrigger:
    def __init__(self):
    
        # trigger parameters:
        self.z_range = [0,1]
        self.x_range = [0,1]
        self.y_range = [0,1]
        self.r_range = [0,1]
        self.volume_type = 'cylinder'
        self.refractory_time = 10
        self.last_time = time.time()
        
    def inrange(self, var, testrange):
        if var > testrange[0] and var < testrange[1]:
            return True
        else:
            return False

    def check_volume(self, pos3d):
    
        x = pos3d[0]
        y = pos3d[1]
        z = pos3d[2]
        
        send_trigger = 0        
        current_time = time.time()
    
        if self.volume_type == 'cylinder':
            r = np.sqrt( x**2 + y**2 )
            if self.inrange(z, self.z_range):
                if self.inrange(r, self.r_range):
                    send_trigger = 1

        # check refractory time:
        if send_trigger == 1:
            if current_time - self.last_time < self.refractory_time:
                send_trigger = 0
                
        if send_trigger == 1:
            self.last_time = time.time()
            return True
        else:
            return False
            


class Listener:
    def __init__(self, trigger_function=example_trigger_function):
    
        self.volume_trigger = VolumeTrigger()
        self.trigger_name = 'volume_trigger'
        self.trigger_function = trigger_function
    
    def run(self):
        # ROS
        rospy.init_node('listener', anonymous=True)
        self.pub = rospy.Publisher(self.trigger_name, Bool)
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.callback)
        rospy.spin()

    def callback(self, super_packet):
        for packet in super_packet.packets:
            for obj in packet.objects:
                position = [obj.position.x, obj.position.y, obj.position.z]
                send_trigger = self.volume_trigger.check_volume(position)
                        
                if send_trigger == 1:
                    self.pub.publish(1)
                    self.trigger_function(obj)
                    self.last_time = time.time()
                else:
                    self.pub.publish(0)
            
    

if __name__ == '__main__':
    listener = Listener(example_trigger_function)
    listener.volume_trigger.z_range = [-0.15, 0]
    listener.volume_trigger.r_range = [0, 0.07]
    listener.volume_trigger.volume_type = 'cylinder'
    listener.volume_trigger.refractory_time = 5
    listener.run()    
    
    
    
    
    
    

