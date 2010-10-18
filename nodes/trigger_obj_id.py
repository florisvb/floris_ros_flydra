#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
import volume_trigger

obj_id_picker = volume_trigger.ObjIDPicker()

listener = volume_trigger.Listener(trigger_function=obj_id_picker.trigger_obj_id_picker, empty_function=obj_id_picker.empty_function)
listener.trigger_name = 'obj_id_trigger'
listener.volume_trigger.z_range = [-0.15, 0.02]
listener.volume_trigger.r_range = [0, 0.07]
listener.volume_trigger.volume_type = 'cylinder'
listener.volume_trigger.refractory_time = 5
listener.run()  
