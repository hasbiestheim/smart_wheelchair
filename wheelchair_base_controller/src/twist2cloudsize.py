#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
from geometry_msgs.msg import Twist
import math
import dynamic_reconfigure.client

class Twist2Cloud:
    def __init__(self):
        # Define listener
        self.stopTime = float(rospy.get_param('~stopTime', '0.20'))
        self.widthInflation = float(rospy.get_param('~angularInflation', '0.5'))
        self.lengthInflation = float(rospy.get_param('~transInflation', '2.0'))
        
	self.clientWidth = dynamic_reconfigure.client.Client("passthrough_x")
	self.clientLength = dynamic_reconfigure.client.Client("passthrough_xx")

        rospy.Subscriber('cmd_vel', Twist, self.processTwist)
        rospy.spin()

    # Go through and modify this joystick and publish a new one
    def processTwist(self,data):
	# If the kinect can see beside, start changing the turning size here.
	#if(self.dynReg):
	# lookBesideDist = data.angular.z * self.stopTime + self.widthInflation
	#params = { 'filter_limit_min' : -lookBesideDist, 'filter_limit_max' : lookBesideDist }
	#self.clientWidth.update_configuration(params)
	  
	# Modify the Kinect look ahead
	lookAheadDist = data.linear.x * self.stopTime + self.lengthInflation
	params = { 'filter_limit_min' : 0.0, 'filter_limit_max' : lookAheadDist }
	self.clientLength.update_configuration(params)
	
if __name__ == '__main__':
  rospy.init_node('twist2joy')
  try:
      cloudMaker = Twist2Cloud()
  except rospy.ROSInterruptException: pass
