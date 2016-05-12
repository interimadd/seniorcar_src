#!/usr/bin/env python
# coding: UTF-8

import rospy
from geometry_msgs.msg import Twist


STRAIGHT_TIME    = 5.0
TURNIG_TIME      = 5.0
TARGET_ANGLE_VEL = 0.1

PROCESSING_RATE = 20.0

if __name__ == '__main__':

	rospy.init_node('generate_cmd_vel')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	cmd_vel = Twist()
	passed_time = 0
	PERIOD = 1.0 / PROCESSING_RATE

	rate = rospy.Rate(PROCESSING_RATE)

	while passed_time < STRAIGHT_TIME:
		cmd_vel.linear.x = 1.0
		cmd_vel.angular.z = 0.0
		pub.publish(cmd_vel)
		rate.sleep()
		passed_time += PERIOD

	passed_time = 0

	while passed_time < TURNIG_TIME:
		cmd_vel.linear.x = 1.0
		cmd_vel.angular.z = 0.1
		pub.publish(cmd_vel)
		rate.sleep()
		passed_time += PERIOD

	passed_time = 0

	while passed_time < 1.0:
		cmd_vel.linear.x = 0.0
		cmd_vel.angular.z = 0.0
		pub.publish(cmd_vel)
		rate.sleep()
		passed_time += PERIOD
