#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
import numpy
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
	
	def subscribe_cmd_vel(self):
		rospy.Subscriber("cmd_vel", Twist, self.cmd_velCallback)

	def cmd_velCallback(self,msg):

		tmp_accel_opening = (msg.linear.x / seniorcar_max_vel)*100
		if tmp_accel_opening < 0:
			tmp_accel_opening = 0
		elif tmp_accel_opening > 99:
			tmp_accel_opening = 99

		self.seniorcar_command.accel_opening = tmp_accel_opening


		if msg.linear.x > 0.001:
			# 目標角速度から操舵角を計算。35度の範囲に収める
			self.seniorcar_command.steer_angle = numpy.clip(math.atan(msg.angular.z * WHEEL_BASE / msg.linear.x) * 180.0 / 3.1415 , -35.0 ,35.0)
			#print self.seniorcar_command
			#self.seniorcar_command.steer_angle = 0


		#print tmp_accel_opening,msg.linear.x,msg.angular.z


	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(20)
		while not rospy.is_shutdown():
			self.pub.publish(self.seniorcar_command)
			rate.sleep()


if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_cmd_vel()
	calclater.calculate_and_publish_voltage()
