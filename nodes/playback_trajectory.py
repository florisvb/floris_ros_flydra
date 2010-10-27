#!/usr/bin/env python
# By Floris van Breugel
# ROS function to play back a previously recorded flydra trajectory, intended for use with the liveviewer function
# Supports flydra stimulus xml files 
# Plotting is done at very low resolution to reduce resource consumption. See 'resolution' in class Object, and 'scene.anti_aliasing_frames' in Viewer

# TODO: 
# 1. add show-cameras option, will require some sort of h5 or calibration file..

import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
import ros_flydra.msg as msgs
import geometry_msgs.msg as geometry_msgs
import time
import sys
import numpy
import numpy as np

from optparse import OptionParser

sys.path.append("/usr/share/pyshared/flydra/a2")
sys.path.append("/home/floris/src/analysis")

import flydra_floris_analysis as ffa


# h5 file, object id, framerate
# or a dataset instance

#source = '/home/floris/data/windtunnel/SA1/checkerboard/DATA20101023_135300.h5'
#obj = 6061


def send_trajectory_to_player(source, obj, num, fps):
    trajectory = ffa.load_trajectory(source, obj, num)
    trajectoryplayer = TrajectoryPlayer(trajectory, fps=fps) 
    return trajectoryplayer

class TrajectoryPlayer:
    def __init__(self, trajectory, fps=None):
        
        self.trajectory = trajectory 
        if fps is None:
            self.fps = self.trajectory.fps
        else:
            self.fps = fps
            
        self.posvel_covariance_diagonal = [0 for i in range(6)]
            
        self.pub = rospy.Publisher("flydra_mainbrain_super_packets", msgs.flydra_mainbrain_super_packet)
        rospy.init_node('trajectoryplayer', anonymous=True)
            
        
    def play(self, repeat=True, pause=30):
        # pause should be equivalent to the ghost tail length in liveview, if that's what you're using this for
        
        repetitions = 1
        if repeat is True:
            repetitions = 1000000
        elif repeat is False:
            repetitions = 1
        else:
            repetitions = repeat
            
        rep = 0
        while rep < repetitions and not rospy.is_shutdown():
            rep += 1
        
            r = rospy.Rate(self.fps)        
            for frame in range(self.trajectory.length):
                if rospy.is_shutdown():        
                    break
                acquire_stamp = rospy.Time.now()
                position = self.trajectory.positions[frame,:]
                velocity = self.trajectory.velocities[frame,:]
                
                flydra_object = msgs.flydra_object( self.trajectory.obj_id, 
                                                    geometry_msgs.Point(position[0], position[1], position[2]), 
                                                    geometry_msgs.Vector3(velocity[0], velocity[1], velocity[2]), 
                                                    self.posvel_covariance_diagonal)
                flydra_objects = [flydra_object]
                
                framenumber = frame
                reconstruction_stamp = rospy.Time.now()
                objects = flydra_objects
                flydra_mainbrain_packet = msgs.flydra_mainbrain_packet(framenumber, reconstruction_stamp, acquire_stamp, objects)
                flydra_mainbrain_super_packet = msgs.flydra_mainbrain_super_packet([flydra_mainbrain_packet])
                print rep, frame
                self.pub.publish(flydra_mainbrain_super_packet)
                r.sleep()
            for i in range(pause):
                if rospy.is_shutdown():        
                    break
                acquire_stamp = rospy.Time.now()
                position = self.trajectory.positions[-1,:]
                velocity = self.trajectory.velocities[-1,:]
                
                flydra_object = msgs.flydra_object( self.trajectory.obj_id, 
                                                    geometry_msgs.Point(position[0], position[1], position[2]), 
                                                    geometry_msgs.Vector3(velocity[0], velocity[1], velocity[2]), 
                                                    self.posvel_covariance_diagonal)
                flydra_objects = [flydra_object]
                
                framenumber = frame
                reconstruction_stamp = rospy.Time.now()
                objects = flydra_objects
                flydra_mainbrain_packet = msgs.flydra_mainbrain_packet(framenumber, reconstruction_stamp, acquire_stamp, objects)
                flydra_mainbrain_super_packet = msgs.flydra_mainbrain_super_packet([flydra_mainbrain_packet])
                print rep, frame
                self.pub.publish(flydra_mainbrain_super_packet)
                r.sleep()

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--source", type="str", dest="source", default=None,
                        help="source file to replay: can be a ffa.dataset class or an h5 file")
    parser.add_option("--obj", type="int", dest="obj", default=None,
                        help="object id number (int) to replay")                    
    parser.add_option("--num", type="int", dest="num", default=1,
                        help="if using a ffa.dataset class, refers to the prefix value")
    parser.add_option("--fps", type="float", dest="fps", default=None,
                        help="playback rate (defaults to original recording rate)")
    parser.add_option("--repeat", type="int", dest="repeat", default=True,
                        help="number of times to repeat the animation")
    
    (options, args) = parser.parse_args()
        
    trajectoryplayer = send_trajectory_to_player(   source = options.source,
                                                    obj = options.obj,
                                                    num = options.num,
                                                    fps = options.fps)
    
    trajectoryplayer.play(repeat=options.repeat)
    
    
    
    
    
    
    
    

