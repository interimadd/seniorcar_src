#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
import numpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from ultimate_seniorcar.msg import SeniorcarState

WHEEL_BASE = 0.9
seniorcar_max_vel = 2.0 / 3.6

class CalculateVoltage:

	seniorcar_command = SeniorcarState()
	seniorcar_command.accel_opening = 0
	seniorcar_command.steer_angle = 0
	seniorcar_command.max_velocity = 2.0

	const_A = 0.0
	new_steer_angle = 0.0

	is_detect_object = False

	def __init__(self):
		rospy.init_node('cmd_vel_to_seniorcar_command')
		self.pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)
	
	def subscribe_cmd_vel(self):
		rospy.Subscriber("cmd_vel", Twist, self.cmd_velCallback)
		rospy.Subscriber("detect_front_object",Bool, self.detectObjectCallback)

	def cmd_velCallback(self,msg):

		self.seniorcar_command.vehicle_velocity = msg.linear.x

		if msg.linear.x < seniorcar_max_vel:

			tmp_accel_opening = int((msg.linear.x / seniorcar_max_vel)*127)
			if tmp_accel_opening < 0:
				tmp_accel_opening = 0
			elif tmp_accel_opening > 127:
				tmp_accel_opening = 127

			self.seniorcar_command.accel_opening = tmp_accel_opening
			self.seniorcar_command.max_velocity = 2.0

		elif msg.linear.x >= seniorcar_max_vel:

			self.seniorcar_command.accel_opening = 127
			self.seniorcar_command.max_velocity = msg.linear.x * 3.6


		if msg.linear.x > 0.01:
			# 目標角速度から操舵角を計算。35度の範囲に収める
			self.new_steer_angle = numpy.clip(math.atan(msg.angular.z * WHEEL_BASE / msg.linear.x) * 180.0 / 3.1415 , -35.0 ,35.0)

		# 前方に障害物を検知した場合は停止する
		if self.is_detect_object:
			self.seniorcar_command.accel_opening = 0

	def detectObjectCallback(self,msg):
		self.is_detect_object = msg.data

	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			# cmd_velの値が離散的すぎるので適当にローパスフィルタを入れた
			self.seniorcar_command.steer_angle = self.const_A * self.seniorcar_command.steer_angle + ( 1.0 - self.const_A ) * self.new_steer_angle
			self.pub.publish(self.seniorcar_command)
			rate.sleep()


if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_cmd_vel()
	calclater.calculate_and_publish_voltage()
