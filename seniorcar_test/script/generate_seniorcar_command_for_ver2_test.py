#!/usr/bin/env python
# coding: UTF-8

#sin波を生成する

import rospy
from geometry_msgs.msg import Twist
from ultimate_seniorcar.msg import SeniorcarState
import numpy
import math


PROCESSING_RATE = 50.0
CYCLE_TIME = 10
CYCLE_NUM  = 4

MAX_STEER_ANGLE_DEG = 10

if __name__ == '__main__':

	rospy.init_node('generate_seniorcar_command')
	pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)

	rate = rospy.Rate(PROCESSING_RATE)
	pub_command = SeniorcarState()

	count = 0
	passed_time = 0

	while not rospy.is_shutdown() and passed_time < CYCLE_TIME * CYCLE_NUM :

		pub_command.steer_angle = MAX_STEER_ANGLE_DEG * math.sin( 2.0 * math.pi * passed_time / CYCLE_TIME)
		pub.publish(pub_command)
		passed_time += 1 / PROCESSING_RATE
		rate.sleep()
