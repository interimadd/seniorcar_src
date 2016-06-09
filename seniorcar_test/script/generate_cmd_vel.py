#!/usr/bin/env python
# coding: UTF-8

import rospy
from geometry_msgs.msg import Twist


STRAIGHT_TIME    = 5.0
TURNIG_TIME      = 3.0
STRAIGHT_TIME_2  = 3.0
STRAIGHT_TIME_3  = 5.0

#TARGET_ANGLE_VEL = 0.4044
#TARGET_ANGLE_VEL = 0.196
TARGET_ANGLE_VEL = 0.6415
TARGET_VEHICLE_VEL = 1.0

PROCESSING_RATE = 50.0

if __name__ == '__main__':

	rospy.init_node('generate_cmd_vel')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	cmd_vel = Twist()
	passed_time = 0
	PERIOD = 1.0 / PROCESSING_RATE

	rate = rospy.Rate(PROCESSING_RATE)

	while passed_time < STRAIGHT_TIME:
		cmd_vel.linear.x = TARGET_VEHICLE_VEL
		cmd_vel.angular.z = 0.0
		pub.publish(cmd_vel)
		rate.sleep()
		passed_time += PERIOD

	passed_time = 0

	while passed_time < TURNIG_TIME:
		cmd_vel.linear.x = TARGET_VEHICLE_VEL
		cmd_vel.angular.z = TARGET_ANGLE_VEL
		pub.publish(cmd_vel)
		rate.sleep()
		passed_time += PERIOD

	passed_time = 0

	while passed_time < STRAIGHT_TIME_2:
		cmd_vel.linear.x = TARGET_VEHICLE_VEL
		cmd_vel.angular.z = 0.0
		pub.publish(cmd_vel)
		rate.sleep()
		passed_time += PERIOD

	passed_time = 0

	while passed_time < TURNIG_TIME:
		cmd_vel.linear.x = TARGET_VEHICLE_VEL
		cmd_vel.angular.z = -TARGET_ANGLE_VEL
		pub.publish(cmd_vel)
		rate.sleep()
		passed_time += PERIOD

	passed_time = 0

	while passed_time < STRAIGHT_TIME_3:
		cmd_vel.linear.x = TARGET_VEHICLE_VEL
		cmd_vel.angular.z = 0.0
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
