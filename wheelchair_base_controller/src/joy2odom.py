#!/usr/bin/env python
import roslib; roslib.load_manifest('wheelchair_base_controller')
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from joy.msg import Joy


class Joy2Odom:
	def __init__(self):
		# Define listener
		rospy.init_node('listener')
		port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
		self.port = serial.Serial(port_name, 115200)
		rospy.Subscriber('joy', Joy, self.processJoy)
        self.x = 0
        self.y = 0
        self.th = 0
        self.baseSpeed = 7
        self.v = .45*self.baseSpeed
        self.track = 1.2
        self.w = .15 * 2*self.baseSpeed / self.track 
        self.rate = 20
		rospy.spin()

	# Go through and modify this joystick and publish a new one
	def processJoy(self,data):
        cmdVx = data.axes[1]
        cmdWz = data.axes[0]
        throttle = .5-.45*data.axes[2]

        vel_est = self.v * cmdVx * throttle
        w_est = self.w * cmdWz * throttle

        self.x = self.x + math.cos(self.th)*vel_est*1/self.rate
        self.y = self.y + math.sin(self.th)*vel_est*1/self.rate
        self.th = self.th + cmdWz * 1/self.rate

        current_time = rospy.Time.now()
	    quaternion = tf.transformations.quaternion_about_axis(self.th, (0,0,1))
	    tf_br.sendTransform(translation = (self.x, self.y, 0), 
			    rotation = tuple(quaternion),
			    time = current_time,
			    child = 'base_link',
			    parent = 'odom')
	    odom_msg = Odometry()
	    odom_msg.header.stamp = current_time
	    odom_msg.header.frame_id = 'odom'

	    odom_msg.pose.pose.position.x = self.x
	    odom_msg.pose.pose.position.y = self.y
	    odom_msg.pose.pose.position.z = 0.0
	    odom_msg.pose.pose.orientation = Quaternion(*quaternion)

	    odom_msg.pose.covariance[0] =  4 # x var
	    odom_msg.pose.covariance[7] =  4 #y var
	    odom_msg.pose.covariance[35] = .4 # theta var

	    odom_msg.child_frame_id = 'base_link'
	    odom_msg.twist.twist.linear.x = pose.vel
	    odom_msg.twist.twist.angular.z = pose.omega

	    #TODO Here starts the constant multiplier noise. Needs real variance...
	    odom_msg.twist.covariance[0] = vel_est
	    odom_msg.twist.covariance[35] = w_est

	    odom_pub.publish(odom_msg)
        

if __name__ == '__main__':
	try:
		chairFaker = Joy2Odom()
	except rospy.ROSInterruptException: pass
