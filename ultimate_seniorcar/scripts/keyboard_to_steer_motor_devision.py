#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import Int8
from keyboard.msg import Key

UP    = 273
DOWN  = 274
RIGHT = 275
LEFT  = 276
SPACE = 32

OUT_VOL = [1,2,3,4]

class CalculateVoltage:

	motor_devision = Int8()
	vol_index = 2

	def __init__(self):
		rospy.init_node('keyboard_to_devision')
		self.pub = rospy.Publisher('input_devision', Int8, queue_size=10)
		self.motor_devision.data = 0
	
	def subscribe_key_data(self):
		rospy.Subscriber("keyboard/keydown", Key, self.keyboarddownCallback)
		rospy.Subscriber("keyboard/keyup", Key, self.keyboardupCallback)

	def keyboarddownCallback(self,key):
		if key.code == RIGHT:
			self.motor_devision.data = 12
		elif key.code == LEFT:
			self.motor_devision.data = -12
		elif key.code == SPACE:
			self.vol_index += 1
			if self.vol_index > 3:
				self.vol_index = 0
			rospy.loginfo("vel_change_to %d",self.vol_index)
		else:
			self.motor_devision.data = 0

	def keyboardupCallback(self,key):
		self.motor_devision.data = 0

	def calculate_and_publish_voltage(self):
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			self.pub.publish(self.motor_devision)
			rate.sleep()

if __name__ == '__main__':
	calclater = CalculateVoltage()
	calclater.subscribe_key_data()
	calclater.calculate_and_publish_voltage()
