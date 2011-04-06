#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
from geometry_msgs.msg import Twist
from joy.msg import Joy
import math

class Twist2Joy:
    def __init__(self):
        # Define listener
        self.vmax = float(rospy.get_param('~invacareVMax', '3.14'))
        self.wmax = float(rospy.get_param('~invacareWMax', '2.15'))

        rospy.Subscriber('cmd_vel', Twist, self.processTwist)
        self.joy_pub = rospy.Publisher('cmd_joy', Joy)
        rospy.spin()

    # Go through and modify this joystick and publish a new one
    def processTwist(self,data):
	axis0 = data.angular.z / self.wmax
	if(axis0 > 1.0):
	  axis0 = 1.0
	elif(axis0 < -1.0):
	  axis0 = -1.0
	  
	axis1 = data.linear.x / self.vmax
	if(axis1 > 1.0):
	  axis1 = 1.0
	elif(axis1 < -1.0):
	  axis1 = -1.0
        
        joy_msg = Joy()
        joy_msg.axes.append(axis0)
        joy_msg.axes.append(axis1)

        self.joy_pub.publish(joy_msg)

if __name__ == '__main__':
  rospy.init_node('twist2joy')
  try:
      controlFaker = Twist2Joy()
  except rospy.ROSInterruptException: pass
