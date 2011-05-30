#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
from joy.msg import Joy

class JoyCalib:
    def __init__(self):
        self.vcmd = 0.0
	self.wcmd = 0.0
        self.joy_pub = rospy.Publisher('joy', Joy)
        rospy.Subscriber('joy_cal', Joy, self.processJoy)
        while not rospy.is_shutdown():
	  self.sendJoy(self.wcmd,self.vcmd)
	  rospy.sleep(0.05)
	
	  
    def sendJoy(self,w,v):
	joy_msg = Joy()
	
	joy_msg.axes.append(w)
	joy_msg.axes.append(v)

	self.joy_pub.publish(joy_msg)
	
    def processJoy(self,data):
	self.vcmd = data.axes[1]
	self.wcmd = data.axes[0]
        


if __name__ == '__main__':
  rospy.init_node('joyrepeater')
  try:
      controlMaker = JoyCalib()
  except rospy.ROSInterruptException: pass
