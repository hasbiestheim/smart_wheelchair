#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
roslib.load_manifest('wheelchair_base_controller')
import rospy
from joy.msg import Joy
import math
from phidgets.msg import encoder_params
import csv

class JoyCalib:
    def __init__(self):
        self.joy_pub = rospy.Publisher('joy', Joy)
        rospy.Subscriber('/phidgets/encoder/102232', encoder_params, self.processLEnc)
	rospy.Subscriber('/phidgets/encoder/102172', encoder_params, self.processREnc)
	self.waitTime = 10.0 # Time to wait in seconds after each command
        
        self.lenc = 0
        self.renc = 0
        self.sendJoy(0,0)
        rospy.sleep(3.0)
	
	self.sendJoy(0.0,-1.0)
	rospy.sleep(2.0*self.waitTime)
	
	with open('speeds_vs_cmd.csv', 'wb') as f:
	  mywriter = csv.writer(f)
	  
	  
	  # Do all commands from -1.0 to 1.0 in steps of .1 for velocity
	  for i in range(0,21):
	    print("At point %s of 21, forward." % (i+1))
	    self.sendJoy(0,-1.0 + (2.0/20.0)*i)
	    le = self.lenc
	    re = self.renc
	    rospy.sleep(self.waitTime)
	    dencV = (((self.lenc-le)+(self.renc-re))/2.0)/self.waitTime
	    dencW = ((self.renc-re)-(self.lenc-le))/self.waitTime
	    writeout = [-1.0 + (2.0/20.0)*i, 0, dencV, dencW]
	    mywriter.writerow(writeout)
	    
	    
	  # Now do it for angular velocity
	  self.sendJoy(-1,0)
	  rospy.sleep(20.0)
	  for i in range(0,21):
	    print("At point %s of 21, angular." % (i+1))
	    self.sendJoy(-1.0 + (2.0/20.0)*i,0)
	    le = self.lenc
	    re = self.renc
	    rospy.sleep(self.waitTime)
	    dencV = (((self.lenc-le)+(self.renc-re))/2.0)/self.waitTime
	    dencW = ((self.renc-re)-(self.lenc-le))/self.waitTime
	    writeout = [0, -1.0 + (2.0/20.0)*i, dencV, dencW]
	    mywriter.writerow(writeout)
	    
	  # Now make a map of all the parts
	  self.sendJoy(-1,-1)
	  rospy.sleep(20.0)
	  k = 1
	  for i in range(0,21): # linear
	    for j in range(0,21): # angular
	      print("At point %s of 441, grid." % (k))
	      k = k + 1
	      self.sendJoy(-1.0+(2.0/20.0)*j,-1.0+(2.0/20.0)*i)
	      if(j == 0): # Stupid wheelchair sucks at getting to its rotational speeds
		rospy.sleep(self.waitTime)
	      le = self.lenc
	      re = self.renc
	      rospy.sleep(self.waitTime)
	      dencV = (((self.lenc-le)+(self.renc-re))/2.0)/self.waitTime
	      dencW = ((self.renc-re)-(self.lenc-le))/self.waitTime
	      writeout = [-1.0+(2.0/20.0)*i, -1.0+(2.0/20.0)*j, dencV, dencW]
	      mywriter.writerow(writeout)
	  print("All done!")
	  
    def sendJoy(self,w,v):
	joy_msg = Joy()
	
	joy_msg.axes.append(w)
	joy_msg.axes.append(v)

	self.joy_pub.publish(joy_msg)
	
    def processLEnc(self,data):
	self.lenc = data.count
    
    def processREnc(self,data):
	self.renc = -data.count
        


if __name__ == '__main__':
  rospy.init_node('joycalibration')
  try:
      controlMaker = JoyCalib()
  except rospy.ROSInterruptException: pass
