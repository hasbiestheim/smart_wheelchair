#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright (c) 2010, Chad Rockey
# All rights reserved
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

from numpy import *
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Quaternion

class EKF:    
  def __init__(self, gX):
    # Kalman process noise
    q = array((1e-8, 1e-8, 1e-8, .10, .10, .10))
    self.Q = diagflat(q)
    
    # Sensor variances
    
    
    # Model constants
    self.dt = 1.0/50.0
    
    # Linear acceleration and braking
    self.ax = 1.0 # try [.9 1.0 1.1 1.2]
    self.bx = 2.1 # try [2.0 2.1 2.2]
    
    # Angular acceleration and braking
    self.aphz = 0.643
    self.bphz = 7.5

    self.initCommon()

  def initCommon(self):
    # Kalman state variables
    self.x = zeros((6,1)) # Kalman state [x; y; th; v; w; ax]
    self.P = eye(6) # Covariance matrix
    self.cmd = Twist()
    
    rospy.init_node('otto_ekf')
    rospy.Subscriber('cmd_vel', Twist, self.record_command)
    rospy.Subscriber('imu', Imu, self.update_filter)
    self.odom_pub = rospy.Publisher('odom', Odometry)
    self.tf_br = tf.TransformBroadcaster()
    rospy.spin()
    
  def record_command(self, msg):
    self.cmd = msg
	      
  def update_filter(self, msg):
    self.modelJacobian()
    self.prediction(self)
    
    self.sensor_update(msg)
    
    self.publish_filter()
      
    
  def prediction(self, msg):
    # Model prediction
    x = self.x[0]
    y = self.x[1]
    th = self.x[2]
    v = self.x[3]
    w = self.x[4]
  
    # Control prediction
    a = self.returnAccelFromV(self.cmd.linear.x, v, self.ax, self.bx, self.dt)
    #alpha = self.returnAccelFromV(self.cmd.angular.z, w, self.aphz, self.bphz, self.dt)
    
    if(abs(self.cmd.linear.x) < 0.05 and abs(a < 0.05)):
      v = 0.0
    
    # Actual state predictions
    self.x[0] = x + v*self.dt*cos(th)
    self.x[1] = y + v*self.dt*sin(th)
    self.x[2] = th + w*self.dt
    self.x[3] = v + a*self.dt
    self.x[4] = w# + alpha*self.dt
    self.x[5] = a
    
    # Covariance prediction
    self.P = dot(dot(self.A,self.P),self.A.T) + self.Q
    
  def returnAccelFromV(self, cmd, current, accel, braking, dt):
    dv = cmd - current
    if(dv >= 0):
      return accel
      '''if(dv > accel*dt):
	return accel
      else:
	return dv/dt'''
    else:
      return -braking
      '''if(dv < -braking*dt):
	return -braking
      else:
	return dv/dt'''
    
  def modelJacobian(self):
    th = self.x[2]
    v = self.x[3]
    a = self.x[4]
    dt = self.dt
    
    self.A = eye(6)
    self.A[0,2] = -v*dt*sin(th)
    self.A[0,3] = dt*cos(th)
    self.A[1,2] = v*dt*cos(th)
    self.A[1,3] = dt*sin(th)
    self.A[2,4] = dt
    self.A[3,5] = dt
    
  def sensor_update(self, msg):
    self.sensorJacobian()
    
    # Assume that our angular velocity is the magnitude of the vector and in the direction of y
    wm = sign(msg.angular_velocity.y)*sqrt(pow(msg.angular_velocity.x,2)+pow(msg.angular_velocity.y,2)+pow(msg.angular_velocity.z,2))
    
    # Assume our acceleration is the magnitude of the vector (excluding x!!!) minus gravity
    
    '''amag = pow(msg.linear_acceleration.y,2)+pow(msg.linear_acceleration.z,2)-pow(9.734,2)
    if(amag < 0):
      amag = 0.0 # Beware of imaginary numbers caused by accelerometer noise/poor scaling
    am = -sign(msg.linear_acceleration.z)*sqrt(amag)
    print am'''
    am = -msg.linear_acceleration.z-0.17
    print am
    
    z = array([wm, am]).T
    
    R = diagflat(array([0.1, 0.1]))
    y = z - self.sensor_prediction()
    S = dot(dot(self.H,self.P),self.H.T) + R
    K = dot(dot(self.P,self.H.T),linalg.pinv(S))
    self.x = self.x + dot(K,y.T)
    self.P = dot(eye(6)-dot(K,self.H),self.P)
      
    
  def sensor_prediction(self):
    w = self.x[4]
    a = self.x[5]
    z = array([w, a])
    return z.T
    
  @staticmethod
  def variance_from_measurement(meas, c0, c1, c2):
    return c0 + c1 * abs(meas) + c2 * pow(meas,2)
    
  def sensorJacobian(self):
    self.H = zeros((2,6))
    self.H[0,4] = 1.0
    self.H[1,5] = 1.0
    
  def getState(self):
    return self.x
    
  def getCovariance(self):
    return self.P
    
  def publish_filter(self):    

		
    x = self.x[0]
    y = self.x[1] # Change to ROS
    th = self.x[2] # Change to ROS
    v = self.x[3]
    w = self.x[4] # Change to ROS
    
    current_time = rospy.Time.now()
    quaternion = tf.transformations.quaternion_about_axis(th, (0,0,1))
    self.tf_br.sendTransform(translation = (x, y, 0), 
		    rotation = tuple(quaternion),
		    time = current_time,
		    child = 'base_link',
		    parent = 'odom')

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = 'odom'

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = 0.0
    odom_msg.pose.pose.orientation = Quaternion(*quaternion)

    odom_msg.pose.covariance[0] = self.P[0,0]
    odom_msg.pose.covariance[7] = self.P[1,1]
    odom_msg.pose.covariance[35] = self.P[2,2]

    odom_msg.child_frame_id = 'base_link'
    odom_msg.twist.twist.linear.x = v
    odom_msg.twist.twist.angular.z = w

    odom_msg.twist.covariance[0] = self.P[3,3]
    odom_msg.twist.covariance[35] = self.P[4,4]

    self.odom_pub.publish(odom_msg)


if __name__ == '__main__':
	try:
	  myEKF = EKF(None)

	except rospy.ROSInterruptException: pass