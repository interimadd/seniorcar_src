#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from ultimate_seniorcar.msg import SeniorcarState


class TopicSubscirber:

	seniorcar_state = SeniorcarState()
	seniorcar_command = SeniorcarState()
	devision = Int8()
	arduino_str = String()
	recive_str = String()

	def __init__(self):
		rospy.init_node('output_to_console')

	def subscribe_tpics(self):
		rospy.Subscriber("seniorcar_state", SeniorcarState, self.stateCallback)

	def stateCallback(self,data):
		self.seniorcar_state = data

	def console_print(self):
		rate = rospy.Rate(50)
		start_time = rospy.get_rostime()
		
		while not rospy.is_shutdown():
			now = rospy.get_rostime()
			print "%d.%09d,%f,%f,%f" % ( now.secs - start_time.secs ,now.nsecs , self.seniorcar_state.steer_angle ,self.seniorcar_state.accel_opening*127, self.seniorcar_state.max_velocity)
			rate.sleep()


if __name__ == '__main__':
	subs = TopicSubscirber()
	subs.subscribe_tpics()
	subs.console_print()
