#!/usr/bin/env python
import wx
from subprocess import Popen

class icreate_start(wx.Frame):
 
    def __init__(self, *args, **kwds):
        kwds["style"] = wx.DEFAULT_FRAME_STYLE
        wx.Frame.__init__(self, *args, **kwds)
        self.icreatepic = wx.StaticBitmap(self, -1, wx.Bitmap("irobot_create_pic.jpg", wx.BITMAP_TYPE_ANY))
        self.lauch_roscore = wx.Button(self, -1, "Launch Roscore")
        self.bringup_simulator = wx.Button(self, -1, "Simulate ICreate")
        self.keyboard_teleop = wx.Button(self, -1, "Keyboard Teleop")
        self.joystick_teleop = wx.Button(self, -1, "Joystick Teleop")
        self.hokuyo = wx.Button(self, -1, "Lidar")
        self.label_1_copy = wx.StaticText(self, -1, ".")
        self.label_1_copy_1 = wx.StaticText(self, -1, ".")
        #self.exit = wx.Button(self, -1, "Exit")
        self.__set_properties()
        self.__do_layout()
 
        self.Bind(wx.EVT_BUTTON, self.launchroscore, self.lauch_roscore)
        self.Bind(wx.EVT_BUTTON, self.simulate, self.bringup_simulator)
        self.Bind(wx.EVT_BUTTON, self.teleopkeyboard, self.keyboard_teleop)
        self.Bind(wx.EVT_BUTTON, self.teleopJoystick, self.joystick_teleop)
    	self.Bind(wx.EVT_BUTTON, self.lidar, self.hokuyo)
        #self.Bind(wx.EVT_BUTTON, self.terminate, self.exit)
 
    def __set_properties(self):
        self.SetTitle("ICreate Control Center")
 
    def __do_layout(self):
        grid_sizer_1 = wx.FlexGridSizer(3, 3, 0, 0)
        grid_sizer_1.Add(self.icreatepic, 0, wx.EXPAND, 0)
        grid_sizer_1.Add(self.lauch_roscore, 0, wx.EXPAND, 0)
        grid_sizer_1.Add(self.bringup_simulator, 0, wx.EXPAND, 0)
        grid_sizer_1.Add(self.keyboard_teleop, 0, wx.EXPAND, 0)
        grid_sizer_1.Add(self.joystick_teleop, 0, wx.EXPAND, 0)
        grid_sizer_1.Add(self.hokuyo, 0, wx.EXPAND, 0)
        grid_sizer_1.Add(self.label_1_copy, 0, 0, 0)
        grid_sizer_1.Add(self.label_1_copy_1, 0, 0, 0)
        #grid_sizer_1.Add(self.exit, 0, wx.EXPAND, 0)
        self.SetSizer(grid_sizer_1)
        grid_sizer_1.Fit(self)
        self.Layout()
 
    def launchroscore(self, event):
	global roscore_process
	command = ('roscore')
	roscore_process = Popen(command)
	event.Skip()
 
    def simulate(self, event):
        global bringup_process
        command = ('rosrun','stage','stageros','`rospack find stage_controllers`/world/roomba-wander.world')
        bringup_process = Popen(command)
        event.Skip()
 
    def teleopkeyboard(self, event):
        global keyboard_process
        command = ('roslaunch','icreate_teleop','keyboard_teleop.launch')
        keyboard_process = Popen(command)
        event.Skip()
 
    def teleopJoystick(self, event):
        global joystick_process
        command = ('roslaunch','icreate_teleop','joystick_teleop.launch')
        joystick_process = Popen(command)
        event.Skip()
 
    def lidar(self, event):
        global calibration_process
        command = ('rosrun','hokuyo_node','hokuyo_node')
        calibration_process = Popen(command)
        event.Skip()
 
   # def terminate(self, event): # wxGlade: icreate_start.<event_handler>
#    try:
 #       roscore_process.terminate()
  #  except:
  #      print "process not running"
  #  try:
  #      bringup_process.terminate()
  #  except:
  #      print "process not running"
  #  try:
  #      keyboard_process.terminate()
  #  except:
  #      print "process not running"
  #  try:
  #      joystick_process.terminate()
  #  except:
  #      print "process not running"
  #  try:
  #      calibration_process.terminate()
  #  except:
  #      print "process not running"
  #  ICreateStartup.Destroy()
  #      event.Skip()
 
# end of class icreate_start
 
if __name__ == "__main__":
    app = wx.PySimpleApp(0)
    wx.InitAllImageHandlers()
    ICreateStartup = icreate_start(None, -1, "IRobot Control Center")
    app.SetTopWindow(ICreateStartup)
    ICreateStartup.Show(True)
    app.MainLoop()
