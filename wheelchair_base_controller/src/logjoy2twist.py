#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
from joy.msg import Joy
from geometry_msgs.msg import Twist
from numpy import *
import scipy.interpolate

class Joy2Twist:
  def __init__(self):
    self.vmax = float(rospy.get_param('~cmdVMax','1.2'))
    self.wmax = float(rospy.get_param('~cmdWMax','1.0'))
    self.revmax = float(rospy.get_param('~cmdRevMax', '0.5'))
    self.joyDeadBand = float(rospy.get_param('~invacareJoyDeadband', '0.12'))
    self.cenc = 0.00115
    self.track = 0.5969
    
    self.cmdVec = arange(-1.0,1.15,0.1)
    
    self.vslope = [717.4377,718.9711,720.1474,720.3939,723.5430,722.9053,717.8307,704.6623,681.2114,645.7175,598.1197,640.5660,676.4702,702.9623,716.8088,722.5974,723.6175,720.8281,720.9342,719.4202,718.6289,718.6289]
    self.voffset = [3.7579, 5.2895, 4.6158,4.7026,4.5711,4.1000,4.3579,3.2868,3.6289,3.6094,3.5500,2.8969,3.5184,3.0684,4.2605,4.5842,4.6658,4.7658,4.6474,5.0079,4.6947,4.6947]
    
    self.wslope = [712.9281,711.1930,713.4281,714.1579,716.8561,713.8649,711.1333,697.5228,672.9298,637.0475,591.3961,640.5722,676.6088,699.6158,711.3702,716.7000,716.2158,714.3456,713.6491,710.9404,712.3474,712.3474]
    self.woffset = [-1.7474,-0.9053,-0.8737,-1.6895,-0.3579,0.1737,-1.0895,0.0789,0.1368,-0.1125,-1.1250,-0.4875,0.3105,-0.6474,-1.2263,-0.1842,-0.6789,-1.9211,-1.4053,-1.4316,-1.5947,-1.5947]
    
    self.vm = scipy.interpolate.interp1d(self.cmdVec, self.vslope)
    self.vb = scipy.interpolate.interp1d(self.cmdVec, self.voffset)
    self.wm = scipy.interpolate.interp1d(self.cmdVec, self.wslope)
    self.wb = scipy.interpolate.interp1d(self.cmdVec, self.woffset)
    
    # Define listener
    rospy.Subscriber('joy', Joy, self.processJoy)
    self.pub = rospy.Publisher('cmd_vel', Twist)
    rospy.spin()

  # Go through and modify this joystick and publish a new one
  def processJoy(self,data):
    cmdVx = data.axes[1]
    cmdWz = data.axes[0]
    
    v_est = 0.0
    w_est = 0.0
    if(abs(cmdVx) > self.joyDeadBand or abs(cmdWz) > self.joyDeadBand):
      v_est = (self.vm(cmdWz)*cmdVx+self.vb(cmdWz))*self.cenc
      if(abs(cmdVx) > 0.9 and abs(cmdWz) > 0.2):
	v_est = v_est * 0.95
      w_est = (self.wm(cmdVx)*cmdWz+self.wb(cmdVx))*self.cenc/self.track
    
    twpub = Twist()
    twpub.linear.x = v_est
    twpub.angular.z = w_est
    self.pub.publish(twpub)
    
    
if __name__ == '__main__':
  rospy.init_node('joy2twist')
  try:
    joyInterface = Joy2Twist()
  except rospy.ROSInterruptException: pass
