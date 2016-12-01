#!/usr/bin/env python
# coding: UTF-8

import rospy
from keyboard.msg import Key
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

UP    = 273
DOWN  = 274
RIGHT = 275
LEFT  = 276
SPACE = 32

class CalculateVoltage:

	pub_twist = Twist()
	pub_joy = Joy()

	def __init__(self):
		rospy.init_node('keyboard_to_cmd_vel')
		self.pub = rospy.Publisher('joy', Joy, queue_size=10)
	
	def subscribe_key_data(self):
		rospy.Subscriber("keyboard/keydown", Key, self.keyboarddownCallback)
		rospy.Subscriber("keyboard/keyup", Key, self.keyboardupCallback)

	def keyboarddownCallback(self,key):
		if key.code == RIGHT:
			self.pub_twist.angular.z = -2
		elif key.code == LEFT:
			self.pub_twist.angular.z = 2
		elif key.code == UP:
			self.pub_twist.linear.x = 0.5
		elif key.code == DOWN:
			self.pub_twist.linear.x = -0.5
		elif key.code == SPACE:
			self.pub_twist.linear.x = 0
			self.pub_twist.angular.z = 0

	def keyboardupCallback(self,key):
		if key.code == UP or key.code == DOWN:
			self.pub_twist.linear.x = 0
		if key.code == RIGHT or key.code == LEFT:
			self.pub_twist.angular.z = 0

	def publish_cmd_vel(self):
		rate = rospy.Rate(20)
		self.pub_joy.axes = [1.0,1.0,1.0,1.0]
		while not rospy.is_shutdown():
			self.pub.publish(self.pub_joy)
			rate.sleep()

if __name__ == '__main__':
	calclater = CalculateVoltage()
	#calclater.subscribe_key_data()
	calclater.publish_cmd_vel()
