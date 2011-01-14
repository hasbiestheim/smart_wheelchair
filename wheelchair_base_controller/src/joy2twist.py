#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
from joy.msg import Joy
from geometry_msgs.msg import Twist

class Joy2Twist:
  def __init__(self):
    self.vmax = 1.2
    self.wmax = 1.0
    self.revmax = 0.3
    
    N = 30 # Average last N commands
    self.vF = []
    self.wF = []
    for i in range(N):
      self.vF.append(0.0)
      self.wF.append(0.0)
    
    # Define listener
    rospy.init_node('joy2twist')
    rospy.Subscriber('joy', Joy, self.processJoy)
    self.pub = rospy.Publisher('cmd_vel', Twist)
    rospy.spin()

  # Go through and modify this joystick and publish a new one
  def processJoy(self,data):
    cmdVx = data.axes[1]
    if(cmdVx >= 0):
      cmdVx = cmdVx*abs(self.vmax)
    else:
      cmdVx = cmdVx*abs(self.revmax)
    
    cmdWz = data.axes[0] * abs(self.wmax)
    
    lpfCmd = self.LowPassFilter(cmdVx,cmdWz)
    
    twpub = Twist()
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
  try:
    joyInterface = Joy2Twist()
  except rospy.ROSInterruptException: pass
