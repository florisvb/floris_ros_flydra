#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
import volume_trigger

SA1 = volume_trigger.SA1()

r = 0.01912/2.
buff = 0.009

listener = volume_trigger.Listener(trigger_function=SA1.trigger_SA1)
listener.trigger_name = 'SA1'
listener.volume_trigger.z_range = [-0.15, 0]
listener.volume_trigger.r_range = [0, r+buff]
listener.volume_trigger.volume_type = 'cylinder'
listener.volume_trigger.refractory_time = 5
listener.run()  
