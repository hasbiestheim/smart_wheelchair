#!/usr/bin/env python
import roslib; roslib.load_manifest('sonar_joy')
import rospy
from sonar_joy.msg import Sonar
from joy.msg import Joy

class JoyLimiter:
	def __init__(self):

		self.linear_Gain = .3
		self.angular_Gain = 1
		self.steering_Gain = .8

		self.EFx = 0.0
		self.ETx = 0.0
		self.forces = [0.0,0.0]
		self.torques = [0.0,0.0]

		# Define listener
		rospy.init_node('listener')
		rospy.Subscriber('sonar', Sonar, self.processSonar)
		rospy.Subscriber('joy', Joy, self.processJoy)

		self.pub = rospy.Publisher('sonar_Joy', Joy)	
		rospy.spin()

	def processSonar(self,data):
		dist = data.range
		if(data.range != 0):
			m = 1/dist
			if(data.header.frame_id == "frontLeftSonar"):
				dx = 0.4826
				dy = 0.2540
				self.forces[0] = -m
				self.torques[0] = dy*m
			elif(data.header.frame_id == "frontRightSonar"):
				dx = 0.4826
				dy = -0.2540
				self.forces[1] = -m
				self.torques[1] = dy*m
		self.EFx = sum(self.forces)
		self.ETx = sum(self.torques)

	# Go through and modify this joystick and publish a new one
	def processJoy(self,data):
		cmdVx = data.axes[1]
		cmdWz = data.axes[0]
		outputVx = self.linear_Gain*self.EFx*abs(cmdVx)+cmdVx
		outputWz = self.angular_Gain*self.ETx*abs(cmdWz)+cmdWz - self.steering_Gain*self.ETx*abs(cmdVx)
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
