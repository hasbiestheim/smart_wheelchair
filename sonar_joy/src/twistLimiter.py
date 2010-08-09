#!/usr/bin/env python
import roslib; roslib.load_manifest('chad_sonar')
import rospy
from chad_sonar.msg import Sonar
from geometry_msgs.msg import Twist

class TwistLimiter:
	def __init__(self):

		self.linear_Gain = 1.0
		self.angular_Gain = 1.0
		self.steering_Gain = 1.0

		self.EFx = 0.0
		self.ETx = 0.0
		self.forces = [0.0,0.0]
		self.torques = [0.0,0.0]

		# Define listener
		rospy.init_node('listener')
		rospy.Subscriber('sonar', Sonar, self.processSonar)
		rospy.Subscriber('rawTwistJoy', Twist, self.processTwist)

		self.pub = rospy.Publisher('mod_Twist', Twist)	
		rospy.spin()

	def processSonar(self,data):
		dist = data.range
		if(data.range != 0):
			m = 1/dist;
			if(data.header.frame_id == "frontLeftSonar"):
				dx = 0.4826;
				dy = -0.2540;
				self.forces[0] = -m;
				self.torques[0] = -dy*m;
			elif(data.header.frame_id == "frontRightSonar"):
				dx = 0.4826;
				dy = 0.2540;
				self.forces[1] = -m;
				self.torques[1] = -dy*m;
		self.EFx = sum(self.forces)
		self.ETx = sum(self.torques)

	# Go through and modify this twist and publish a new one
	def processTwist(self,data):
		cmdVx = data.linear.x
		cmdWz = data.angular.z
		outputVx = (self.linear_Gain*self.EFx+1)*cmdVx
		outputWz = (self.angular_Gain*self.ETx+1)*cmdWz + self.steering_Gain*self.ETx*cmdVx
		message = TwistLimiter.sendTwist(outputVx,outputWz)
		self.pub.publish(message)

	@staticmethod
	def sendTwist(linearVx,angularWz):
		twist = Twist()
		twist.linear.x = linearVx
		twist.angular.z = angularWz
		return twist

if __name__ == '__main__':
	try:
		listener = TwistLimiter()

	except rospy.ROSInterruptException: pass
