#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('wheelchair_base_controller')
import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion
from joy.msg import Joy
from numpy import sign

class Joy2Odom:
    def __init__(self):
        # Define listener
        rospy.init_node('odometer')
        rospy.Subscriber('joy', Joy, self.processJoy)
        self.odom_pub = rospy.Publisher('odom', Odometry)
        self.tf_br = tf.TransformBroadcaster()
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.vmax = 3.000
        self.wmax = 0.7577
        self.v = 0.0
        self.w = 0.0
        self.a = 1.00
        self.aw = 0.9472
        self.ba = 0.392
        self.bw = 1.106
        self.dt = 1.0/20.0
        self.lastTime = rospy.get_time()
        rospy.spin()
        

    # Go through and modify this joystick and publish a new one
    def processJoy(self,data):
        cmdVx = data.axes[1]
        cmdWz = data.axes[0]
        throttle = 1.0#.5-.45*data.axes[2]
        
        current_time = rospy.get_time()
        dt = float(current_time - self.lastTime)

        self.v = self.calculateVelocity(self.v, cmdVx*self.vmax*throttle, self.a, self.ba, dt)
        self.w = self.calculateVelocity(self.w, cmdWz*self.wmax, self.aw, self.bw, dt)

	if(abs(self.w) < .001):
	  self.x += math.cos(self.th)*self.v*dt
	  self.y += math.sin(self.th)*self.v*dt
	  self.th += self.w*dt
	else:
	  r = self.v/self.w
	  thp = self.th + self.w*dt
	  self.x += -r*math.sin(self.th)+r*math.sin(thp)
	  self.y += r*math.cos(self.th)-r*math.cos(thp)
	  self.th = thp
        
        self.lastTime = current_time

        current_time = rospy.Time.now()
        quaternion = tf.transformations.quaternion_about_axis(self.th, (0,0,1))
        self.tf_br.sendTransform(translation = (self.x, self.y, 0), 
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
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = self.w

        #TODO Here starts the constant multiplier noise. Needs real variance...
        odom_msg.twist.covariance[0] = 1
        odom_msg.twist.covariance[35] = 1

        self.odom_pub.publish(odom_msg)
        
    # Calculate an expected velocity command
    def calculateVelocity(self, current, target, accel, brake, dt):
      if(target >= current and current >= 0.0): # Positive acceleration, positive velocity
	preV = current + dt*accel
	if(preV > target):
	  return target
	else:
	  return preV
      elif(target < current and current > 0.0): # Negative acceleration, positive velocity
	preV = current - dt*brake
	if(preV < 0.0):
	  return 0.0
	else:
	  return preV
      elif(target <= current and current <= 0.0): # Positive acceleration, positive velocity
	preV = current - dt*accel
	if(preV < target):
	  return target
	else:
	  return preV
      elif(target > current and current < 0.0): # Negative acceleration, positive velocity
	preV = current + dt*brake
	if(preV > 0.0):
	  return 0.0
	else:
	  return preV

if __name__ == '__main__':
    try:
        chairFaker = Joy2Odom()
    except rospy.ROSInterruptException: pass
