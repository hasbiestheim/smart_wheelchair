#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
from joy.msg import Joy
from geometry_msgs.msg import Twist
import math

class Joy2Twist:
  def __init__(self):
    self.vmax = float(rospy.get_param('~cmdVMax','1.2'))
    self.wmax = float(rospy.get_param('~cmdWMax','1.0'))
    self.revmax = float(rospy.get_param('~cmdRevMax', '0.5'))
    joyDeadBand = float(rospy.get_param('~invacareJoyDeadband', '0.118'))
    self.minvx = joyDeadBand*self.vmax
    self.minwz = joyDeadBand*self.wmax
    self.smooth = bool(rospy.get_param('~smoothJoy', 'false'))
    
    N = 20 # Average last N commands
    self.vF = []
    self.wF = []
    for i in range(N):
      self.vF.append(0.0)
      self.wF.append(0.0)
    
    # Define listener
    rospy.Subscriber('joy', Joy, self.processJoy)
    self.pub = rospy.Publisher('cmd_vel', Twist)
    rospy.spin()

  # Go through and modify this joystick and publish a new one
  def processJoy(self,data):
    cmdVx = data.axes[1]
    if(cmdVx >= 0):
      cmdVx = cmdVx*self.vmax
    else:
      cmdVx = cmdVx*self.revmax
    if(abs(cmdVx) < self.minvx):
      cmdVx = 0.0
    
    cmdWz = data.axes[3]
    cmdWz = cmdWz * self.wmax
    if(abs(cmdWz)  < self.minwz):
      cmdWz = 0.0
    
    twpub = Twist()
    if(self.smooth == False):
      twpub.linear.x = cmdVx
      twpub.angular.z = cmdWz
    else:
      lpfCmd = self.LowPassFilter(cmdVx,cmdWz)
      twpub.linear.x = lpfCmd[0]
      twpub.angular.z = lpfCmd[1]

    self.pub.publish(twpub)
    
    
  def LowPassFilter(self, cmdVx, cmdWz):
    cmdOut = []
    self.vF.pop(0)
    self.vF.append(cmdVx)
    cmdOut.append(float(sum(self.vF))/len(self.vF))
    
    self.wF.pop(0)
    self.wF.append(cmdWz)
    cmdOut.append(float(sum(self.wF))/len(self.wF))
    return cmdOut
    
if __name__ == '__main__':
  rospy.init_node('joy2twist')
  try:
    joyInterface = Joy2Twist()
  except rospy.ROSInterruptException: pass
