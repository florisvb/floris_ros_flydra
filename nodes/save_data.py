#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
import time
import time
import wx
import pickle

import numpy as np

import sys
sys.path.append("/home/floris/src/floris")

from optparse import OptionParser

class Frame(wx.Frame):
    def __init__(self):
        wx.Frame.__init__(self, None, title='Flydra Save Data GUI')
        panel = wx.Panel(self)
        
        # buttons
        self.storedata = wx.Button(self, id=-1, label='store data',
            pos=(8, 8), size=(175, 28))
        self.storedata.Bind(wx.EVT_BUTTON, self.storedataclick)
        
        self.savedata = wx.Button(self, id=-1, label='save data',
            pos=(8, 48), size=(175, 28))
        self.savedata.Bind(wx.EVT_BUTTON, self.savedataclick)
        
        self.cleardata = wx.Button(self, id=-1, label='clear data',
            pos=(8, 88), size=(175, 28))
        self.cleardata.Bind(wx.EVT_BUTTON, self.cleardataclick)
        
        # default parameters        
        self.reset()

        self.Show(True)
        
        ################################################
        # ROS stuff
        rospy.init_node('save_data_gui', anonymous=True)
        rospy.Subscriber("flydra_mainbrain_super_packets", flydra_mainbrain_super_packet, self.flydradata)
        #################################################
        
    def reset(self):
        self.flypos = None
        self.data = None
        
    def storedataclick(self,event):
        if self.flypos == None:
            print 'No data saved (no fly or too many flies)!'
            return
        else:
            print 'saving data: ', self.flypos
            if self.data is not None:
                self.data = np.vstack( (self.data, self.flypos) )
            else:
                self.data = np.array(self.flypos)
                
    def savedataclick(self,event):
        filename = time.strftime("flydra_data_%Y%m%d_%H%M%S",time.localtime())
        print 'saving calibration to file: ', filename
        fname = (filename)  
        fd = open( fname, mode='w' )
        pickle.dump(self.data, fd)
        return 1
        
    def cleardataclick(self,event):
        print 'clearing data'
        self.reset()
                
    def flydradata(self, super_packet):
        now = time.time()
        for packet in super_packet.packets:
            if len(packet.objects) > 1:
                self.flypos = None
            else:
                for obj in packet.objects:
                    self.flypos = [obj.position.x, obj.position.y, obj.position.z]

        
if __name__ == '__main__':
    app = wx.App()
    f = Frame()
    app.MainLoop()
    
    
    
    
    
