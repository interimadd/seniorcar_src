#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import Int8
from ultimate_seniorcar.msg import SeniorcarState


class CalculateVoltage:

	motor_devision = Int8()
	seniorcar_state = SeniorcarState()

	def __init__(self):
		rospy.init_node('target_angle_to_devision')
		self.pub = rospy.Publisher('input_devision', Int8, queue_size=10)
		self.motor_devision.data = 0
	
	def subscribe_key_data(self):
		rospy.Subscriber("seniorcar_command", SeniorcarState, self.keyboarddownCallback)
		rospy.Subscriber("seniorcar_state", SeniorcarState, self.updateSeniorcarStateData)


	def keyboarddownCallback(self,data):
		difference = data.steer_angle - seniorcar_state.steer_angle

		"""
		if difference > 5:
			self.motor_devision.data = 12
		elif difference < -5:
			self.motor_devision.data = -12
		else:
			self.motor_devision.data = 0
		"""

		self.motor_devision.data = difference


	def updateSeniorcarStateData(self,data):
		seniorcar_state = data


	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.motor_devision)
			rate.sleep()

if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_key_data()
	calclater.calculate_and_publish_voltage()
