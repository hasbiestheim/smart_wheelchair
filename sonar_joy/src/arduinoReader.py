#!/usr/bin/env python
import roslib; roslib.load_manifest('sonar_joy')
import rospy
import serial
from sonar_joy.msg import Sonar

def sendMessage(sonarId, distance):
	sonarDictionary = {'3':"frontLeftSonar", '2':"frontRightSonar"}
	sonarName = sonarDictionary[sonarId]
	sonar = Sonar()
	sonar.header.stamp = rospy.Time.now()
	sonar.header.frame_id = sonarName
	sonar.range = float(distance)
	sonar.beam_angle = .3489
	sonar.max_range = 6.45
	sonar.min_range = .1524
	return sonar

def talker():
	pub = rospy.Publisher('sonar', Sonar)
	rospy.init_node('sonarReader')
	port_name = rospy.get_param("~port_name", "/dev/ttyUSB0")
	port = serial.Serial(port_name, 115200)
	for i in range(0,10):
		line = port.readline()

	while not rospy.is_shutdown():
		line = port.readline()
		#print repr(line)
		chunks = line.split(",")
		if len(chunks) == 2:
			#print "Length was 2"
			message = sendMessage(chunks[0],chunks[1])
			pub.publish(message)
	port.close()

if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass
