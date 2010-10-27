#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time
from std_msgs.msg import *
import socket
import numpy as np
import cPickle as pickle

import sys
sys.path.append('/home/floris/src/flydra/flydra')
import common_variables

def test_function(obj):
    pass

class ObjIDPicker:
    def __init__(self, function=test_function):
        self.pref_obj_id = None
        self.volume_checker = VolumeChecker()
        self.function = function
        
        r = 0.01912/2.
        buff = 0.08
        inner_buff = 0.006

        self.volume_checker.z_range = [-0.15, 0.08]
        self.volume_checker.r_range = [0, r+buff]
        self.volume_checker.volume_type = 'cylinder'

    def run(self):
        # ROS
        rospy.init_node('obj_id_picker', anonymous=True)
        self.pub = rospy.Publisher("flydra_pref_obj_id", Bool)
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.callback)
        rospy.spin()
        
    def new_pref_obj_id(self, obj_d):
        self.pref_obj_id = obj_id
        self.pub.publish(UInt32(self.pref_obj_id))
        print 'new obj id: ', self.pref_obj_id
            
    def callback(self, super_packet):
        for packet in super_packet.packets:
            current_obj_ids = [obj.obj_id for obj in packet.objects]
            
            # case: We still have Pref Obj ID
            if self.pref_obj_id in current_obj_ids: # check volume
                # find index of pref_obj_id:
                pref_obj = packet.objects[current_obj_ids.index(self.pref_obj_id)]
                in_volume = self.volume_checker.check_volume(pref_obj)
                if in_volume:
                    self.finish(pref_obj)
                else:
                    self.new_pref_obj_id(None)
                    
            # case: We do not have Pref Obj ID
            if self.pref_obj_id is None:
                for obj in packet.objects:
                    in_volume = self.volume_checker.check_volume(obj)
                    if in_volume:
                        self.new_pref_obj_id(obj.obj_id)
                        self.finish(obj)
            
            # if no objects in volume, choose None
            self.new_pref_obj_id(None)
            self.finish(None)
            
    def finish(self, obj):
        # here is where you feed a highspeed camera trigger function or VR function etc.
        self.function(obj)
        return 1
                    
                    
                    
            
class VolumeChecker:
    def __init__(self):
    
        # trigger parameters:
        self.z_range = [0,0]
        self.x_range = [0,0]
        self.y_range = [0,0]
        self.r_range = [0,0]
        self.volume_type = 'cylinder'
        
        self.speed_range = [0,0]
        self.check_speed = False
        
    def inrange(self, var, testrange):
        if var > testrange[0] and var < testrange[1]:
            return True
        else:
            return False

    def check_volume(self, obj):
    
        pos3d = [obj.position.x, obj.position.y, obj.position.z]
    
        x = pos3d[0]
        y = pos3d[1]
        z = pos3d[2]
        
        in_volume = 0  
        
               
        # VOLUME
        if self.volume_type == 'cylinder':
            r = np.sqrt( x**2 + y**2 )
            if self.inrange(z, self.z_range):
                if self.inrange(r, self.r_range):
                    in_volume = 1
                    print 'involume'
            else: 
                in_volume = 0
            
        # SPEED  
        if self.check_speed:    
            speed = np.linalg.norm([obj.velocity.x, obj.velocity.y, obj.velocity.z])
            if not self.inrange(speed, self.speed_range):
                in_volume = 0
        

        if in_volume == 1:
            return True
        else:
            return False
            

class SA1:
    def __init__(self):
        self.hostname = '127.0.0.1'
        self.sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.save_data = True
        self.data = None
        self.armed = False
        
        self.last_time = 0
        self.refractory_time = 5.*60.
        
        self.filename = time.strftime("SA1_%Y%m%d",time.localtime())
        self.fname = (self.filename)  
        self.fd = open( self.fname, mode='w' )
        
        self.volume_checker = VolumeChecker()
        r = 0.01912/2.
        buff = 0.01
        inner_buff = 0.005
        self.volume_checker.z_range = [-0.14, 0.01]
        self.volume_checker.r_range = [r+inner_buff, r+buff]
        self.volume_checker.speed_range = [0.006, 1]
        self.volume_checker.check_speed = False
        self.volume_checker.volume_type = 'cylinder'
        
    def arm_SA1(self):
        self.sender.sendto('3',(self.hostname,common_variables.trigger_network_socket_port))
        self.armed = True
        print 'armed'
        
    def trigger_SA1(self, obj):
        print 'trigger_called'
        print_time=True
        time_string = time.strftime("%Y%m%d_%H_%M_%S",time.localtime())
        self.sender.sendto('3',(self.hostname,common_variables.trigger_network_socket_port))
        self.armed = False
        if print_time:
            print 'triggered!, time: ' + time_string 
        if self.save_data:
            new_data = np.array([obj.obj_id, time.time()])
            if self.data is None:
                self.data = new_data
            else:
                self.data = np.vstack((self.data, new_data))
            pickle.dump(self.data, self.fd)
            
    def check_trigger(obj):
        if self.armed is False:
            current_time = time.time()
            if current_time-self.last_time > self.refractory_time:
                self.arm_SA1()
            self.last_time = current_time
            
        if obj is not None:
            in_volume = self.volume_checker.check_volume(obj)
            if in_volume:
                self.trigger_SA1(obj)
        
            
                
    

if __name__ == '__main__':

    sa1 = SA1()
    
    obj_id_picker = ObjIDPicker(sa1.check_trigger)
    obj_id_picker.run()    
    
    
    
    
    
    

