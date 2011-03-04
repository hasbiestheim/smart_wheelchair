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
    q = array((1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1, 1, 1, 1, .1))
    self.Q = diagflat(q)
    
    # Sensor variances
    
    
    # Model constants
    self.dt = 1.0/50.0
    
    # Linear acceleration and braking
    self.ax = 1.0 # try [.9 1.0 1.1 1.2]
    self.bx = 2.1 # try [2.0 2.1 2.2]
    
    self.p0 = 0.0 # Level pitch of IMU
    self.r0 = 0.0 # Level roll of IMU

    self.initCommon()

  def initCommon(self):
    # Kalman state variables
    self.x = zeros((11,1)) # Kalman state [x; y; z; roll; pitch; yaw; vel, rollrate, pitchrate, yawrate, accel]
    self.P = eye(11) # Covariance matrix
    self.cmd = Twist()
    
    rospy.Subscriber('cmd_vel', Twist, self.record_command)
    rospy.Subscriber('imu', Imu, self.update_filter)
    self.odom_pub = rospy.Publisher('odom_rpw', Odometry)
    self.tf_br = tf.TransformBroadcaster()
    rospy.spin()
    
  def record_command(self, msg):
    self.cmd = msg
	      
  def update_filter(self, msg):
    self.modelJacobian()
    self.prediction(self)
    
    self.sensor_update(msg)
    
    self.publish_filter()
    
    
  def returnAccelFromV(self, cmd, current, accel, braking, dt):
    dv = cmd - current
    if(dv >= 0):
      return accel
      if(dv > accel*dt):
	return accel
      else:
	return dv/dt
    else:
      return -braking
      if(dv < -braking*dt):
	return -braking
      else:
	return dv/dt
      
    
  def prediction(self, msg):
    # Model prediction
    x = self.x[0]
    y = self.x[1]
    z = self.x[2]
    r = self.x[3]
    p = self.x[4]
    w = self.x[5]
    v = self.x[6]
    rp = self.x[7]
    pp = self.x[8]
    wp = self.x[9]
  
    # Control prediction
    a = self.returnAccelFromV(self.cmd.linear.x, v, self.ax, self.bx, self.dt)
    
    # Clamp v to limit stationary drift
    if(abs(self.cmd.linear.x) < 0.01 and abs(v < 0.05)):
      v = 0.0
    
    # Actual state predictions
    self.x[0] = x + v*self.dt*cos(w)*cos(p)
    self.x[1] = y + v*self.dt*sin(w)*cos(p)
    self.x[2] = z - v*self.dt*sin(p)
    self.x[3] = r + rp*self.dt
    self.x[4] = p + pp*self.dt
    self.x[5] = w + wp*self.dt
    self.x[6] = v + a*self.dt
    self.x[7] = rp
    self.x[8] = pp
    self.x[9] = wp
    self.x[10] = a
    
    # Covariance prediction
    self.P = dot(dot(self.A,self.P),self.A.T) + self.Q

  def modelJacobian(self):
    x = self.x[0]
    y = self.x[1]
    z = self.x[2]
    r = self.x[3]
    p = self.x[4]
    w = self.x[5]
    v = self.x[6]
    rp = self.x[7]
    pp = self.x[8]
    wp = self.x[9]
    a = self.x[10]
    dt = self.dt
    
    self.A = eye(alen(self.x))
    self.A[0,4] = -v*dt*cos(w)*sin(p)
    self.A[0,5] = -v*dt*sin(w)*cos(p)
    self.A[0,6] = dt*cos(w)*cos(p)
    self.A[1,4] = -v*dt*cos(w)*sin(p)
    self.A[1,5] = v*dt*cos(w)*cos(p)
    self.A[1,6] = dt*sin(w)*cos(p)
    self.A[2,4] = v*dt*cos(p)
    self.A[2,6] = -dt*sin(p)
    self.A[3,7] = dt
    self.A[4,8] = dt
    self.A[5,9] = dt
    self.A[6,10] = dt
    
  def sensor_update(self, msg):
    self.sensorJacobian()
    
    #wm = sign(msg.angular_velocity.y)*sqrt(pow(msg.angular_velocity.x,2)+pow(msg.angular_velocity.y,2)+pow(msg.angular_velocity.z,2))    
    #am = -msg.linear_acceleration.z-self.gz
    
    gyros = array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z, 0.0])
    accels = array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z, 0.0])
    imu2otto = tf.transformations.quaternion_from_euler(3.14/2.0,-3.14/2.0,-0.011,'rxyz')
    gyros = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(imu2otto,gyros),tf.transformations.quaternion_inverse(imu2otto))
    accels = tf.transformations.quaternion_multiply(tf.transformations.quaternion_multiply(imu2otto,accels),tf.transformations.quaternion_inverse(imu2otto))
    
    # Magically transform into being in Ottos coordinate frame, instead of crappy IMU/BOX frame
    # Improve this by using the known errors in mounting + some tf
    gyros =  gyros[0:3]
    accels = accels[0:3]
    
    #print gyros
    #print accels
    
    
    #print "g = %.3f" % (sqrt(accels[0]*accels[0]+accels[1]*accels[1]+accels[2]*accels[2]))
    
    z = append(gyros,accels,1).T
    
    R = diagflat(array([0.01, 0.01, 0.01, 0.3, 0.3, 0.3]))
    y = z - self.sensor_prediction()
    S = dot(dot(self.H,self.P),self.H.T) + R
    K = dot(dot(self.P,self.H.T),linalg.pinv(S))
    self.x = self.x + dot(K,y.T)
    self.P = dot(eye(alen(self.x))-dot(K,self.H),self.P)
      
    
  def sensor_prediction(self):
    x = self.x[0]
    y = self.x[1]
    z = self.x[2]
    r = self.x[3]
    p = self.x[4]
    w = self.x[5]
    v = self.x[6]
    rp = self.x[7]
    pp = self.x[8]
    wp = self.x[9]
    a = self.x[10]
    dt = self.dt
    g = 9.77
    
    # Prediction of accelerometers from orientation and acceleration
    ax = a - g * sin(p)
    ay = g * cos(p) * sin(r)
    az = g * cos(p) * cos(r)
    
    z = array([rp, pp, wp, ax, ay, az])
    return z.T
    
  @staticmethod
  def variance_from_measurement(meas, c0, c1, c2):
    return c0 + c1 * abs(meas) + c2 * pow(meas,2)
    
  def sensorJacobian(self):
    x = self.x[0]
    y = self.x[1]
    z = self.x[2]
    r = self.x[3]
    p = self.x[4]
    w = self.x[5]
    v = self.x[6]
    rp = self.x[7]
    pp = self.x[8]
    wp = self.x[9]
    a = self.x[10]
    dt = self.dt
    g = 9.77
    
    self.H = zeros((6,11))
    self.H[0,7] = 1.0
    self.H[1,8] = 1.0
    self.H[2,9] = 1.0
    self.H[3,4] = -g * cos(p)
    self.H[3,10] = 1.0
    self.H[4,3] = g * cos(p) * cos(r)
    self.H[4,4] = -g * sin(p) * sin(r)
    self.H[5,3] = -g * cos(p) * sin(r)
    self.H[5,4] = -g * sin(p) * cos(r)
    
  def getState(self):
    return self.x
    
  def getCovariance(self):
    return self.P
    
  def publish_filter(self):
    x = self.x[0,0]
    y = self.x[1,0]
    z = self.x[2,0]
    r = self.x[3,0]
    p = self.x[4,0]
    w = self.x[5,0]
    v = self.x[6,0]
    rp = self.x[7,0]
    pp = self.x[8,0]
    wp = self.x[9,0]
    a = self.x[10,0]
    
    current_time = rospy.Time.now()
    quaternion = tf.transformations.quaternion_from_euler(r,p,w,'rxyz')
    print "%.3f %.3f %.3f" % (r, p, w)
    #print "%.3f %.3f %.3f" % (r*180.0/3.14, p*180.0/3.14, w*180.0/3.14)
    #tf.transformations.quaternion_about_axis(w, (0,0,1))
    self.tf_br.sendTransform(translation = (x, y, z), 
		    rotation = tuple(quaternion),
		    time = current_time,
		    child = 'base_link_rpw',
		    parent = 'odom')

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = 'odom'

    odom_msg.pose.pose.position.x = x
    odom_msg.pose.pose.position.y = y
    odom_msg.pose.pose.position.z = z
    odom_msg.pose.pose.orientation = Quaternion(*quaternion)

    odom_msg.pose.covariance[0] = self.P[0,0]
    odom_msg.pose.covariance[7] = self.P[1,1]
    odom_msg.pose.covariance[35] = self.P[2,2]

    odom_msg.child_frame_id = 'base_link_rpw'
    odom_msg.twist.twist.linear.x = v
    odom_msg.twist.twist.angular.x = rp
    odom_msg.twist.twist.angular.y = pp
    odom_msg.twist.twist.angular.z = wp

    odom_msg.twist.covariance[0] = self.P[3,3]
    odom_msg.twist.covariance[35] = self.P[4,4]

    self.odom_pub.publish(odom_msg)


if __name__ == '__main__':
  rospy.init_node('otto_ekf_rpw')
  try:
    myEKF = EKF(None)

  except rospy.ROSInterruptException: pass