#!/usr/bin/env python
# By Floris van Breugel
# ROS function to show a live view of the 3D objects being tracked
# Supports flydra stimulus xml files 

# TODO: add show-cameras option, will require some sort of h5 or calibration file..

import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time
import sys
import numpy
import numpy as np

from optparse import OptionParser

from enthought.tvtk.api import tvtk
from enthought.mayavi import mlab
from enthought.tvtk.tools import visual

sys.path.append("/usr/share/pyshared/flydra/a2")
import flydra.a2.xml_stimulus as xml_stimulus

class Object:
    def __init__(self, ghost_length=20):
        self.ghost_length = ghost_length
        # initialize some fly objects
        self.x = np.ones(ghost_length)
        self.y = np.zeros(ghost_length)
        self.z = np.zeros(ghost_length)
        self.speed = np.zeros(ghost_length)
        # View them
        self.l = mlab.points3d(self.x, self.y, self.z, self.speed, scale_factor=0.01)
        # Now animate the data.
        self.ms = self.l.mlab_source
        
    def update(self, x, y, z, speed):
        self.x = np.hstack( ([x],self.x[0:self.ghost_length-1]) )
        self.y = np.hstack( ([y],self.y[0:self.ghost_length-1]) )
        self.z = np.hstack( ([z],self.z[0:self.ghost_length-1]) )
        self.speed = np.hstack( ([speed],self.speed[0:self.ghost_length-1]) )
        self.ms.set(x=self.x, y=self.y, z=self.z, scalars=self.speed)
        
    def hide(self):
        self.speed = np.zeros(self.ghost_length)
        self.ms.set(scalars=self.speed)
        
class Viewer:
    def __init__(self, stim_xml_filename=None, num_points=5):
    
        #stim_xml_filename = '/home/floris/data/calibrations/landingbox_post_fulllength.xml'
    
        # initialize the viewer window
        self.f = mlab.figure(size=(800,600))
        self.f.scene.anti_aliasing_frames = 1
        visual.set_viewer(self.f)
        
        
        ################# moving fly objects ################
        self.num_points = num_points
        self.plot_objects = [Object() for i in range(self.num_points)]
        ######################################################
        
        self.objects = None
        self.speed = np.zeros(self.num_points)
        
        ################### stimulus xml #####################
        if stim_xml_filename is not None:
            actors = []
            stim_xml = xml_stimulus.xml_stimulus_from_filename(stim_xml_filename)
            actors.extend(stim_xml.get_tvtk_actors())
            for a in actors:
                visual.show_actor(a)
        ######################################################
       
        # ROS stuff
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.callback)
        rospy.init_node('listener', anonymous=True)
        
        # run the animation
        self.animate()
        
    def animate(self):
        

        @mlab.show
        @mlab.animate(delay=10)
        def anim():
            while 1:
                self.f.scene.disable_render = True # disable rendering while we calculate the new scene
                for i in range(len(self.plot_objects)):
                
                    if i < len(self.objects):
                        try:
                            if self.objects is not None:
                                self.plot_objects[i].update(self.objects[i].position.x, self.objects[i].position.y, self.objects[i].position.z, self.speed[i])
                        except:
                            continue
                            
                    else:
                        self.plot_objects[i].hide()
                        pass
                self.f.scene.disable_render = False
                        
                yield

        # Run the animation. The strange definition of the function in a function + yield is needed to get the decorators to work. Maybe there's a more elegant way to do this?
        anim()
        
    def callback(self, super_packet):
       
        for packet in super_packet.packets:
            self.objects = packet.objects
            self.data_timestamp = time.time()
            self.latency = self.data_timestamp-packet.acquire_stamp.to_seconds()
            for i in range(len(self.objects)):
                self.speed[i] = (self.objects[i].velocity.x**2 + self.objects[i].velocity.y**2 + self.objects[i].velocity.z**2)**(0.5)

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--stim-xml", type="str", dest="stim_xml_filename", default=None,
                        help="filename for the stimulus xml")
    parser.add_option("--num-points", type="int", dest="num_points", default=5,
                        help="number of flydra objects to display in the live viewer")
    parser.add_option("--ghost-length", type="int", dest="ghost_length", default=10,
                        help="number of ghost frames to display for each object")
    
    (options, args) = parser.parse_args()
        
    viewer = Viewer(stim_xml_filename=options.stim_xml_filename,
                    num_points=options.num_points)
