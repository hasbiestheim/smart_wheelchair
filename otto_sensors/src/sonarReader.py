#!/usr/bin/env python
import roslib; roslib.load_manifest('otto_sensors')
import rospy
import serial
from sensor_msgs.msg import Range

def sendMessage(sonarId, distance):
	sonarDictionary = {'1':"frontLeftSonar", '2':"diagLeftSonar", '3':"sideLeftSonar", '4':"frontRightSonar", '5':"diagRightSonar", '6':"sideRightSonar"}
	sonarName = sonarDictionary[sonarId]
	sonar = Range()
	sonar.header.stamp = rospy.Time.now()
	sonar.header.frame_id = sonarName
	sonar.range = float(distance)
	sonar.field_of_view = .3489
	sonar.max_range = 6.45
	sonar.min_range = .1524
	return sonar

def talker():
	pub = rospy.Publisher('sonars', Range)
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
