#!/usr/bin/env python
# coding: UTF-8

import rospy
from geometry_msgs.msg import Twist
from ultimate_seniorcar.msg import SeniorcarState
import numpy


PROCESSING_RATE = 5.0
PROCESS_TIME = 120.5

STEER_ANGLE_DEG = 5

if __name__ == '__main__':

	rospy.init_node('generate_seniorcar_command')
	pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)

	rate = rospy.Rate(PROCESSING_RATE)
	pub_command = SeniorcarState()

	count = 0

	while not rospy.is_shutdown() and count / PROCESSING_RATE < PROCESS_TIME:

		case = int(count/PROCESSING_RATE)%4

		if case == 0 or case == 2:
			pub_command.steer_angle = 0
		elif case == 1:
			pub_command.steer_angle = STEER_ANGLE_DEG
		elif case == 3:
			pub_command.steer_angle = -STEER_ANGLE_DEG

		pub.publish(pub_command)
		count += 1
		rate.sleep()
