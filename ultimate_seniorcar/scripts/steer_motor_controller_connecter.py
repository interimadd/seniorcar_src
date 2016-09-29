#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
import sys
import numpy
from std_msgs.msg import Int8
from std_msgs.msg import String
from ultimate_seniorcar.msg import SeniorcarState

MAX_ANGLE = 40
GERA_CONSTANT = 3000.0 * 160.0 * 1.561 / 360.0

target_steer_angle = 0.0
state_steer_angle = 0.0
send_str = ""


def callback(data):
	global target_steer_angle
	target_steer_angle = -data.steer_angle

def state_callback(data):
	global state_steer_angle
	state_steer_angle = data.steer_angle

def connect_with_motor_controller():
	global send_str
	port = rospy.get_param('steer_motor_port',"/dev/ttyACM0")
	pub = rospy.Publisher('motor_controller_input', String, queue_size=10)
	pub_str = String()
	print port
	try:
		ser = serial.Serial(port,9600)
		ser.setDTR(False)
		time.sleep(1)
		ser.setDTR(True)
		print "start connection with steer_motor_controller"
	except:
		print "cannot start connection with steer_motor_controller"
		sys.exit()

	time.sleep(10)
	rate = rospy.Rate(20)

	send_str = "LR" + str(int(state_steer_angle * GERA_CONSTANT)) + "\n"
	ser.write(send_str)
	print send_str
	ser.write("M\n")
	time.sleep(1)

	ser.write("HO\n")
	time.sleep(0.5)

	print "finish initializing steer angle origine"

	while  not rospy.is_shutdown():
		send_target_angle_to_steer_motor(ser)
		pub_str.data = send_str
		pub.publish(pub_str)
		rate.sleep()

	ser.close()


def send_target_angle_to_steer_motor(ser):

	global send_str
	target_value = max( -MAX_ANGLE , min( target_steer_angle , MAX_ANGLE ) ) * GERA_CONSTANT
	send_str = "LA" + str(int(target_value)) + "\n" 
	rospy.loginfo(send_str)
	ser.write(send_str)
	ser.write("M\n")
	

if __name__ == '__main__':
	rospy.init_node('steer_motor_controller_connecter')
	rospy.Subscriber("seniorcar_command", SeniorcarState, callback)
	rospy.Subscriber("seniorcar_state", SeniorcarState, state_callback)
	connect_with_motor_controller()
