#!/usr/bin/env python
import roslib; roslib.load_manifest('floris_ros_flydra')
import rospy
from ros_flydra.msg import *
from std_msgs.msg import String

import wx
import time
import numpy as np
import sys

# import some additional paths
#sys.path.append("/home/floris/ros/floris_src")
import DummyFlydra

class DrawPanel(wx.Frame):

    def __init__(self, parent):
        # variables that should be function inputs
        self.dummy = 1

        self.flies = None
        self.noflies = False
        self.obj_ids = None
        self.pref_obj_id = None
        
        wx.Frame.__init__(self, parent, title="Draw on Panel")
        self.SetSize((640, 480))
        self.panel = wx.Panel(self)

        self.trigger_rectangle = None
        
        # menu bar 
        menubar = wx.MenuBar()
        file = wx.Menu()
        quit = wx.MenuItem(file, 1, '&Quit\tCtrl+Q')
        file.AppendItem(quit)
        menubar.Append(file, '&File')
        self.SetMenuBar(menubar)
        self.Bind(wx.EVT_MENU, self.OnQuit, id=1)

        # ROS stuff
        rospy.init_node('flydra_listener', anonymous=True)
        rospy.Subscriber("flydra_data", flydra_packet, self.callback)
        self.pub_pref_obj_id = rospy.Publisher('flydra_pref_obj_id', String)

        # events
        self.panel.Bind(wx.EVT_PAINT, self.paint)
        self.panel.Bind(wx.EVT_MOUSE_EVENTS, self.mouse)
        self.panel.Bind(wx.EVT_KEY_UP, self.keyboard)

    def OnQuit(self, event):
        print 'quitting'
        self.Close()


    def callback(self, data):
        self.flies = data.flies        
        self.obj_ids = [fly.obj_id for fly in self.flies]
        if self.pref_obj_id not in self.obj_ids and self.pref_obj_id is not None:
            self.set_pref_obj_id(None)
            print 'lost the preferred fly! please select a new one... (mouseclick)'

    def mouse(self, event):

        # make draggable rectangle
        if event.RightDown():
            self.trigger_rectangle = [event.GetPositionTuple(), event.GetPositionTuple()]
        if event.Dragging():
            self.trigger_rectangle[1] = event.GetPositionTuple()
        if event.RightUp():
            self.trigger_rectangle[1] = event.GetPositionTuple()
            self.set_pref_obj_id(None)

        if event.LeftDown():
            self.choose_pref_obj_id()

        event.Skip()

    def keyboard(self, event):

        if event.GetKeyCode() == 81: # q
            self.OnQuit(event=None)
        if event.GetKeyCode() == 67: # c
            print 'clearing obj ids and selection rectangle'
            self.set_pref_obj_id(None)
            self.trigger_rectangle = None

    def set_pref_obj_id(self, obj_id):
        self.pref_obj_id = obj_id
        self.pub_pref_obj_id.publish(String(self.pref_obj_id))
        return
            
    def choose_pref_obj_id(self):
        # choose a new object ID!

        # two possibilities: 
        # 1. we have not yet selected a obj id
        # 2. we want to change the current obj id

        # check to make sure there are flies
        if self.noflies is True:
            print 'no flies to select from!'
            return
            
        if self.pref_obj_id is not None:
            index = self.obj_ids.index(self.pref_obj_id)
            new_index = index+1
            if index+1 > len(self.flies)-1:
                new_index = 0
            self.set_pref_obj_id(self.obj_ids[new_index])
            print 'chose a new pref_obj_id: ', self.pref_obj_id
            return

        if self.pref_obj_id is None:
            # choose the oldest obj_id
            self.set_pref_obj_id(self.obj_ids[0])
            print 'chose a new pref_obj_id: ', self.pref_obj_id
            return

    def paint(self, event=None):
        dc = wx.PaintDC(self.panel)
        dc.Clear()
        dc.SetBrush(wx.TRANSPARENT_BRUSH)
        dc.SetPen(wx.Pen(wx.RED, 4))

        # print some basic information
        existing_objs = 'existing objects: ' + str(self.obj_ids)
        dc.DrawText(existing_objs, 0, 0)
        pref_obj_id = 'pref obj id: ' + str(self.pref_obj_id)
        dc.DrawText(pref_obj_id, 0, 20)

        try:
            for fly in self.flies:
                # reproject fly onto camera... 
                if self.dummy:
                    xpos, ypos = DummyFlydra.reproject(fly.state_vecs[0:3])
                    # if xpos, ypos inside rectangle set pref obj id
                    if self.trigger_rectangle is not None and self.pref_obj_id is None:
                        if xpos >= self.trigger_rectangle[0][0] and xpos <= self.trigger_rectangle[1][0]:
                            if ypos >= self.trigger_rectangle[0][1] and ypos <= self.trigger_rectangle[1][1]:
                                self.set_pref_obj_id(fly.obj_id)

                if fly.obj_id == self.pref_obj_id:
                    dc.DrawEllipse(xpos-7,ypos-2,30,20)
                dc.DrawText(str(fly.obj_id), xpos, ypos)

                if self.noflies is True:
                    print 'found a fly!'
                self.noflies = False
        except: 
            if self.noflies is False:
                print 'no flies... waiting for a fly'
                self.noflies = True


        if self.trigger_rectangle is not None:
            width = self.trigger_rectangle[1][0] - self.trigger_rectangle[0][0]
            height = self.trigger_rectangle[1][1] - self.trigger_rectangle[0][1]
            dc.DrawRectangle(self.trigger_rectangle[0][0], self.trigger_rectangle[0][1], width, height)
            
        self.panel.Refresh()
        
if __name__ == '__main__':
    app = wx.App(False)
    frame = DrawPanel(None)
    frame.Show()
    app.MainLoop()


