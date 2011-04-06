#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('wheelchair_base_controller')
import rospy
import serial
import struct
from joy.msg import Joy

class Joy2Chair:
  def __init__(self):
    # Define listener
    port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
    self.port = serial.Serial(port_name, 115200)
    rospy.Subscriber('joy', Joy, self.processJoy)
    rospy.spin()

  # Go through and modify this joystick and publish a new one
  def processJoy(self,data):
    cmdVx = int(127 + data.axes[1]*70.0)
    #print cmdVx
    cmdWz = int(127 - data.axes[0]*70.0)
    # -1 is fastest, +1 is slowest, pushing causes axis to become most negative
    #throttle = .5-.45*data.axes[2] #wheelchair throttle does not accept negative values
    throttle = 255#int(1+throttle * 254) 
    # Send command down to arduino
    print "%s %s %s 0" % (cmdVx, cmdWz, throttle)
    self.port.write(struct.pack(">BBBB", cmdVx, cmdWz, throttle, 0))

if __name__ == '__main__':
  rospy.init_node('joy2chair')
  try:
    chairSender = Joy2Chair()
  except rospy.ROSInterruptException: pass
