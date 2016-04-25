#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from geometry_msgs.msg import Twist
from ultimate_seniorcar.msg import SeniorcarState

WHEEL_BASE = 0.9
seniorcar_max_vel = 2.0 / 3.6

class CalculateVoltage:

	seniorcar_command = SeniorcarState()
	seniorcar_command.accel_opening = 0
	seniorcar_command.steer_angle = 0

	def __init__(self):
		rospy.init_node('cmd_vel_to_seniorcar_command')
		self.pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)
		self.vol.x = MID_VOL
		self.vol.y = MID_VOL
	
	def subscribe_cmd_vel(self):
		rospy.Subscriber("cmd_vel", Twist, self.cmd_velCallback)

	def cmd_velCallback(self,msg):

		tmp_accel_opening = msg.linear.x / seniorcar_max_vel
		if tmp_accel_opening < 0:
			tmp_accel_opening = 0
		else if tmp_accel_opening > 99:
			tmp_accel_opening = 99

		seniorcar_command.accel_opening = tmp_accel_opening


		if msg.linear.x > 0.001:
			seniorcar_command.steer_angle = atan(msg.angular.z * WHEEL_BASE / msg.linear.x) * 180.0 / 3.1415


	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.seniorcar_command)
			rate.sleep()


if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_cmd_vel()
	calclater.calculate_and_publish_voltage()
