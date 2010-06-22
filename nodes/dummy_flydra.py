#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
import ros_flydra.msg as msgs
import geometry_msgs.msg as geometry_msgs

import numpy as np
import time
from optparse import OptionParser
pi = np.pi

# create a dummy mainbrain with arbitrary number of objects, objects are born and killed, and move around

class DummyMainbrain:

    def __init__(self, nobjects=5, latency=0.05, birth_rate=0.02, death_rate=0.02):
        self.max_num_objects = nobjects
        self.latency = latency
        self.prob_birth = birth_rate
        self.prob_death = death_rate
        
        self.newest_object = 0
        self.framenumber = 0
        
        # initial fly collection
        init_objects = np.random.randint(0,self.max_num_objects)
        self.objects = []
        for i in range(init_objects):
            self.newest_object = self.newest_object + 1
            self.objects.append( DummyObject(self.newest_object) )
        
        # ros stuff
        self.pub = rospy.Publisher("flydra_mainbrain_super_packets", msgs.flydra_mainbrain_super_packet)
        rospy.init_node('dummy_mainbrain', anonymous=True)
        
        print 'dummy mainbrain initialized'
        

    def get_objects(self):
        acquire_stamp = rospy.Time.now()
        self.framenumber = self.framenumber + 1
        time.sleep(self.latency)
        
        birth_check = np.random.random()
        if birth_check < self.prob_birth:
            if len(self.objects) < self.max_num_objects:
                self.newest_object = self.newest_object+1
                self.objects.append(DummyObject(self.newest_object))

        death_check = np.random.random()
        if death_check < self.prob_death:
            if len(self.objects) > 1:
                del self.objects [np.random.randint(0,len(self.objects))]

        # package with mainbrain message format
        flydra_objects = []
        for i in range(len(self.objects)):
            obj_id, position, velocity, posvel_covariance_diagonal = self.objects[i].get_state()
            flydra_object = msgs.flydra_object(obj_id, geometry_msgs.Point(position[0], position[1], position[2]), geometry_msgs.Vector3(velocity[0], velocity[1], velocity[2]), posvel_covariance_diagonal)
            flydra_objects.append(flydra_object)
        
        framenumber = self.framenumber
        reconstruction_stamp = rospy.Time.now()
        objects = flydra_objects
        flydra_mainbrain_packet = msgs.flydra_mainbrain_packet(framenumber, reconstruction_stamp, acquire_stamp, objects)
        flydra_mainbrain_super_packet = msgs.flydra_mainbrain_super_packet([flydra_mainbrain_packet])
        
        self.pub.publish(flydra_mainbrain_super_packet)
        return flydra_mainbrain_super_packet
        
    def run(self):
        print 'dummy mainbrain running'
        while not rospy.is_shutdown():
            time.sleep(self.latency)
            self.get_objects()
        
class DummyObject:

    def __init__(self, obj_id):
        # random start position values
        self.x0 = np.random.randn()
        self.y0 = np.random.randn()
        self.z0 = np.random.randn()

        # random amplitude values
        self.xamp = np.random.randn()
        self.yamp = np.random.randn()
        self.zamp = np.random.randn()

        # object ID number
        self.obj_id = obj_id

    def get_state(self):
        theta = time.time() % (2*pi)
        x = self.xamp*np.cos( theta ) + self.x0
        xvel = -self.xamp*np.sin( theta )
        y = self.yamp*np.sin( theta ) + self.y0
        yvel = -self.yamp*np.cos( theta )
        z = self.zamp*np.sin( theta/2.3 ) + self.z0
        zvel = -self.zamp/2.3*np.cos( theta/2.3 )   

        position = [x,y,z]
        velocity = [xvel,yvel,zvel]
        posvel_covariance_diagonal = [0 for i in range(6)]
        
        return self.obj_id, position, velocity, posvel_covariance_diagonal

########### run as mainbrain node #############
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--nobjects", type="int", dest="nobjects", default=5,
                        help="maximum number of objects")
    parser.add_option("--latency", type="float", dest="latency", default=0.05,
                        help="artifial latency of dummy mainbrain")
    parser.add_option("--birth-rate", type="float", dest="birth_rate", default=0.02,
                        help="probability that a new object is born")
    parser.add_option("--death-rate", type="float", dest="death_rate", default=0.02,
                        help="probability that a new object dies")
    (options, args) = parser.parse_args()

    print 'starting dummy mainbrain'
    dummy_mainbrain = DummyMainbrain(   nobjects=options.nobjects, 
                                        latency=options.latency, 
                                        birth_rate=options.birth_rate, 
                                        death_rate=options.death_rate)
    dummy_mainbrain.run()






    
