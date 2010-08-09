#!/usr/bin/env python
import roslib; roslib.load_manifest('sonar_joy')
import rospy
from sonar_joy.msg import Sonar
from joy.msg import Joy

class JoyLimiter:
	def __init__(self):
		self.threshold = .5
		self.dangerL = False
		self.dangerR = False
		# Define listener
		rospy.init_node('listener')
		rospy.Subscriber('sonar', Sonar, self.processSonar)
		rospy.Subscriber('joy', Joy, self.processJoy)

		self.pub = rospy.Publisher('sonar_Joy', Joy)	
		rospy.spin()

	def processSonar(self,data):
		dist = data.range
		if(data.range < self.threshold and data.header.frame_id == "frontLeftSonar"):
			self.dangerL = True
		else:
			self.dangerL = False
		if(data.range < self.threshold and data.header.frame_id == "frontRightSonar"):
			self.dangerR = True
		else:
			self.dangerR = False

	def stopForward(self, vCmd):
		if(vCmd <= 0):
			return vCmd
		elif(dangerL or dangerR):
			return 0
		return vCmd

	# Go through and modify this joystick and publish a new one
	def processJoy(self,data):
		cmdVx = data.axes[1]
		cmdWz = data.axes[0]
		outputVx = stopForward(cmdVx)
		message = JoyLimiter.sendJoy(data,outputVx,outputWz)
		self.pub.publish(message)

	@staticmethod
	def sendJoy(data,linearVx,angularWz):
		joy = data
		tempAxes = list(joy.axes)
		tempAxes[1] = JoyLimiter.checkRange(linearVx)
		tempAxes[0] = JoyLimiter.checkRange(angularWz)
		joy.axes = tempAxes
		return joy

	@staticmethod
	def checkRange(floaty):
		if(floaty > 1.0):
			return 1.0
		elif(floaty < -1.0):
			return -1.0
		return floaty

if __name__ == '__main__':
	try:
		listener = JoyLimiter()

	except rospy.ROSInterruptException: pass
