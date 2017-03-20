#!/usr/bin/env python
# coding: UTF-8

import rospy
from std_msgs.msg import Int8
from std_msgs.msg import String
from ultimate_seniorcar.msg import SeniorcarState


class TopicSubscirber:

	seniorcar_state = SeniorcarState()
	seniorcar_command = SeniorcarState()
	devision = Int8()
	arduino_str = String()
	recive_str = String()

	def __init__(self):
		rospy.init_node('output_to_console')

	def subscribe_tpics(self):
		rospy.Subscriber("seniorcar_command", SeniorcarState, self.commandCallback)
		rospy.Subscriber("seniorcar_state", SeniorcarState, self.stateCallback)
		rospy.Subscriber("motor_controller_input",String,self.arduinoCallback)

	def reciveCallback(self,data):
		self.recive_str = data

	def commandCallback(self,data):
		self.seniorcar_command = data

	def stateCallback(self,data):
		self.seniorcar_state = data

	def devisionCallback(self,data):
		self.devision = data

	def arduinoCallback(self,data):
		self.arduino_str = data

	def console_print(self):
		rate = rospy.Rate(50)
		start_time = rospy.get_rostime()
		print "time,command_angle_deg,state_angle_deg,motor_contoller_command,state_vel"
		
		while not rospy.is_shutdown():
			now = rospy.get_rostime()
			print "%d.%09d,%f,%f,%s,%f,%f" % ( now.secs - start_time.secs ,now.nsecs , self.seniorcar_command.steer_angle ,self.seniorcar_state.steer_angle ,self.arduino_str.data,self.seniorcar_command.vehicle_velocity,self.seniorcar_state.vehicle_velocity)
			rate.sleep()


if __name__ == '__main__':
	subs = TopicSubscirber()
	subs.subscribe_tpics()
	subs.console_print()
