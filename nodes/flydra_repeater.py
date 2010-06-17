#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy

import socket, Queue
import flydra.kalman.data_packets as data_packets
import flydra.common_variables
import Pyro.core
import numpy as np
import time
import sys
from optparse import OptionParser

from ros_flydra.msg import *

def talker(connect_to_mainbrain=True, server=None):

    ###################### variables that should be function inputs ###########
    loginfo = 0

    ###################### initialize #########################################

    # flydra publishers
    pub_flydra = rospy.Publisher('flydra_data', flydra_packet)
    
    rospy.init_node('flydra_repeater')

    # set up connection to mainbrain
    if connect_to_mainbrain:
	    # make connection to flydra mainbrain
	    my_host = '' # get fully qualified hostname
	    my_port = 8320 # arbitrary number
	    # create UDP socket object, grab the port
	    sockobj = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	    print 'binding',( my_host, my_port)
	    sockobj.bind(( my_host, my_port))
	    print 'bound to: ', my_host, my_port

	    # connect to mainbrain
	    mainbrain_hostname = server
	    mainbrain_port = flydra.common_variables.mainbrain_port
	    mainbrain_name = 'main_brain'
	    print 'connecting to mainbrain...'
	    print 'using port: ', mainbrain_port
	    print 'using name: ', mainbrain_name
	    print 'connecting to server: ', server
	    remote_URI = "PYROLOC://%s:%d/%s" % (mainbrain_hostname,
		                             mainbrain_port,
		                             mainbrain_name)
	    Pyro.core.initClient(banner=0)
	    mainbrain = Pyro.core.getProxyForURI(remote_URI)
	    mainbrain._setOneway(['log_message'])
	    my_host_fqdn = socket.getfqdn(my_host)
	    mainbrain.register_downstream_kalman_host(my_host_fqdn,my_port)
	    print 'connection established with ', mainbrain


    # create a dummy mainbrain
    if not connect_to_mainbrain:
        dummy_mainbrain = DummyFlydra.DummyMainbrain()
        print 'connected to dummy_mainbrain'

    ###################### while running #########################################
    
    while not rospy.is_shutdown():

        # use flydra
        if connect_to_mainbrain: 
            print 'waiting for packet'
            buf, addr = sockobj.recvfrom(4096)
            print 'packet recieved'
            packets = data_packets.decode_super_packet( buf )
            
            for packet in packets:
                tmp = data_packets.decode_data_packet(packet)
                # obj_ids: order from one that's been around the longest first
                # state_vecs = [x,y,z,xvel,yvel,zvel]
                # meanPs = error for each object
                corrected_framenumber, timestamp_i, timestamp_f, obj_ids, state_vecs, meanPs = tmp
                latency = timestamp_f - timestamp_i
                print latency

        # use dummy flydra
        if not connect_to_mainbrain:
            latency, obj_ids, state_vecs = dummy_mainbrain.get_flies()
                
        # print list of obj_ids just to make sure things are happening        
        if loginfo:
            rospy.loginfo(obj_ids)

        # package and publish flydra data
        flies_tmp = []
        for i in range(len(obj_ids)):
            fly = flydra_fly(obj_ids[i], state_vecs[i,:]) # package individual fly info into the flydra_fly msg packet
            flies_tmp.append(fly)
        flies = flydra_packet(flies_tmp, latency) # package all the flies and global flydra data into the flydra_packet msg packet
        pub_flydra.publish(flies)
        
if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("--dummy", action="store_false", dest="connect_to_mainbrain", default=True,
                        help="connect to DummyMainbrain instead of mainbrain")
    parser.add_option("--server", type="string", dest="server", default=None)
    (options, args) = parser.parse_args()

    if options.connect_to_mainbrain is False:
        # import some additional paths
        sys.path.append("/home/floris/src/floris")
        import DummyFlydra

    if options.connect_to_mainbrain is True and options.server is None:
        raise ValueError('If using mainbrain, please specify a server')

    try:
        talker(connect_to_mainbrain=options.connect_to_mainbrain, server=options.server)
    except rospy.ROSInterruptException: pass

