#!/usr/bin/env python
# coding: UTF-8

import rospy
from rosgraph_msgs.msg import Clock


class TopicSubscirber:

	print_clock = Clock()

	def __init__(self):
		rospy.init_node('return_time_clock')

	def subscribe_tpics(self):
		rospy.Subscriber("clock", Clock, self.clockCallback)

	def clockCallback(self,data):
		self.print_clock = data

	def console_print(self):
		rate = rospy.Rate(0.5)
		start_time = rospy.get_rostime()
		
		while not rospy.is_shutdown():
			now = rospy.get_rostime()
			print " "
			print "%d.%09d" % ( self.print_clock.clock.secs ,self.print_clock.clock.nsecs)
			rate.sleep()


if __name__ == '__main__':
	subs = TopicSubscirber()
	subs.subscribe_tpics()
	subs.console_print()
