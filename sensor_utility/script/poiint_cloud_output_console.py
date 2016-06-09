#!/usr/bin/env python
# coding: UTF-8

import rospy
from ultimate_seniorcar.msg import SeniorcarState
from sensor_msgs.msg import PointCloud

class TopicSubscirber:

	def __init__(self):
		rospy.init_node('output_to_console')

	def subscribe_tpics(self):
		rospy.Subscriber("laser_point", PointCloud, self.pointCallback)

	def pointCallback(self,data):
		for i in range(0,len(data.points)):
			print "%06f,%06f,%06f" % ( data.points[i].x, data.points[i].y, data.points[i].z )

if __name__ == '__main__':
	subs = TopicSubscirber()
	subs.subscribe_tpics()
	rospy.spin()
