#! /usr/bin/env python
# -*- coding: utf-8 -*-

import roslib
roslib.load_manifest("otto_sensors")
import rospy

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from numpy import *
import tf
import serial

class SF9DOF_Broadcaster:
    def __init__(self, port):
        self.port = port
        #self.mag_err = magneto_err
        #self.gyro_err = gyro_err
        #self.accel_err = accel_err
        self.pub = rospy.Publisher("imu", Imu)
        #self.message = Imu()
        #self.message.header.frame_id = "sf9dof_link"
        #self.message.mag_error = self.mag_err
        #self.message.omega_error = self.gyro_err
        #self.message.accel_error = self.accel_err

    def loop(self):
        while not rospy.is_shutdown():
            line = self.port.readline()
            chunks = line.split(":")
            if chunks[0] == "!QUAT":
                readings = chunks[1].split(",")
                if len(readings) == 10:
		    imu_msg = Imu()
		    imu_msg.header.stamp = rospy.Time.now()
		    imu_msg.header.frame_id = 'imu'
		    imu_msg.orientation.x = float(readings[0])
		    imu_msg.orientation.y = float(readings[1])
		    imu_msg.orientation.z = float(readings[2])
		    imu_msg.orientation.w = float(readings[3])
		    imu_msg.orientation_covariance = list(zeros((3,3)).flatten())
		    imu_msg.angular_velocity.x = float(readings[4])
		    imu_msg.angular_velocity.y = float(readings[5])
		    imu_msg.angular_velocity.z = float(readings[6])
		    imu_msg.angular_velocity_covariance = list(0.1*diagflat(ones((3,1))).flatten())
		    imu_msg.linear_acceleration.x = float(readings[7])
		    imu_msg.linear_acceleration.y = float(readings[8])
		    imu_msg.linear_acceleration.z = float(readings[9])
		    imu_msg.linear_acceleration_covariance = list(0.1*diagflat(ones((3,1))).flatten())
		    self.pub.publish(imu_msg)
		    quaternion = (imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w)
		    tf_br.sendTransform(translation = (0,0, 0), rotation = quaternion,time = rospy.Time.now(),child = 'imu',parent = 'world')
		else:
		    rospy.logerr("Did not get a valid IMU packet, got %s", line)
	    else:
		rospy.loginfo("Did not receive IMU data, instead got %s", line)

if __name__ == "__main__":
    rospy.init_node('imu_quatcaster')
    port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
    port = serial.Serial(port_name, 115200, timeout = 1)
    tf_br = tf.TransformBroadcaster()
    broadcaster = SF9DOF_Broadcaster(port)
    broadcaster.loop()
    port.close()
