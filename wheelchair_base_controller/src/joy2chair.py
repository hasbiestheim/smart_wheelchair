#!/usr/bin/env python
import roslib; roslib.load_manifest('wheelchair_base_controller')
import rospy
import serial
from joy.msg import Joy

class Joy2Chair:
	def __init__(self):
		# Define listener
		rospy.init_node('listener')
		port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
		self.port = serial.Serial(port_name, 115200)
		rospy.Subscriber('joy', Joy, self.processJoy)
		rospy.spin()

	# Go through and modify this joystick and publish a new one
	def processJoy(self,data):
		cmdVx = data.axes[1]
		#print cmdVx
		cmdWz = data.axes[0]
		throttle = abs(data.axes[2]) #wheelchair doesn't go backwards, so don't send me negative throttles
		# Send command down to arduino
		# 254 is max, but need to limit between???  1V and 4V :-P (so like: 56 and 198)
		command_string = "%d %d %d\r" % (127 + cmdVx*71.0,127 - cmdWz*71.0, throttle * 255)
		self.port.write(command_string)

if __name__ == '__main__':
	try:
		chairSender = Joy2Chair()
	except rospy.ROSInterruptException: pass
