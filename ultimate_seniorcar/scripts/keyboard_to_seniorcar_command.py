#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import Int8
from keyboard.msg import Key
from ultimate_seniorcar.msg import SeniorcarState

UP    = 273
DOWN  = 274
RIGHT = 275
LEFT  = 276
SPACE = 32

OUT_VOL = [1,2,3,4]

ANGLE_INCLIMENT = 10.0

class CalculateVoltage:

	motor_devision = Int8()
	seniorcar_command = SeniorcarState()
	seniorcar_command.accel_opening = 0
	seniorcar_command.steer_angle = 0

	def __init__(self):
		rospy.init_node('keyboard_to_devision')
		self.pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)
	
	def subscribe_key_data(self):
		rospy.Subscriber("keyboard/keydown", Key, self.keyboarddownCallback)
		rospy.Subscriber("keyboard/keyup", Key, self.keyboardupCallback)

	def keyboarddownCallback(self,key):
		if key.code == RIGHT:
			self.seniorcar_command.steer_angle -= ANGLE_INCLIMENT
		elif key.code == LEFT:
			self.seniorcar_command.steer_angle += ANGLE_INCLIMENT
		elif key.code == UP:
			self.seniorcar_command.accel_opening = 127
			self.seniorcar_command.max_velocity = 2.0
		elif key.code == DOWN:
			if self.seniorcar_command.direction_switch == 0:
				self.seniorcar_command.direction_switch = 1
			else:
				self.seniorcar_command.direction_switch = 0
		elif key.code == SPACE:
			self.seniorcar_command.accel_opening = 0
			self.seniorcar_command.steer_angle = 0

	def keyboardupCallback(self,key):
		if key.code == UP:
			self.seniorcar_command.accel_opening = 0

	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.pub.publish(self.seniorcar_command)
			rate.sleep()

if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_key_data()
	calclater.calculate_and_publish_voltage()
