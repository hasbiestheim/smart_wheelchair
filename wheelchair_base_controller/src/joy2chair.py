#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('wheelchair_base_controller')
import rospy
import serial
import struct
from joy.msg import Joy
from std_msgs.msg import Bool

class Joy2Chair:
  def __init__(self):
    # Define listener
    port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
    self.port = serial.Serial(port_name, 115200)
    rospy.Subscriber('joy', Joy, self.processJoy)
    rospy.Subscriber('collision_warning', Joy, self.processJoy)
    self.pub = rospy.Publisher("motors_enabled", Bool)
    self.reflexAllow = True
    
  def checksum(self, b1, b2, b3):
    return (~(b1 + b2 + b3)+1) & 0xFF
    
  def processReflex(self,data):
    self.reflexAllow = data

  # Go through and modify this joystick and publish a new one
  def processJoy(self,data):
    cmdVx = int(127 + data.axes[1]*70.0) & 0xFF
    #print cmdVx
    cmdWz = int(127 - data.axes[0]*70.0) & 0xFF
    # -1 is fastest, +1 is slowest, pushing causes axis to become most negative
    #throttle = .5-.45*data.axes[2] #wheelchair throttle does not accept negative values
    throttle = 255 & 0xFF #int(1+throttle * 254) 
    chk = self.checksum(cmdVx,cmdWz,throttle)
    # Send command down to arduino
    rospy.logdebug("Sent commands are: %s %s %s %s", cmdVx, cmdWz, throttle, chk)
    self.port.write(struct.pack(">BBBB", cmdVx, cmdWz, throttle, chk))
    
  def loop(self):
    while not rospy.is_shutdown():
      readLine = self.port.readline()
      if(readLine[0] ==  "1"):
	self.pub.publish(True)
      else:
	self.pub.publish(False)
      

if __name__ == '__main__':
  rospy.init_node('joy2chair')
  try:
    chairSender = Joy2Chair()
    chairSender.loop()
  except rospy.ROSInterruptException: pass
