#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
import sys
from std_msgs.msg import Int8

send_d = 0;

def callback(data):
	global send_d
	send_d = data.data

def connect_with_arduino():
	port = rospy.get_param('steer_motor_port',"/dev/ttyACM0")
	print port
	try:
		ser = serial.Serial(port,9600)
		ser.setDTR(False)
		time.sleep(1)
		ser.setDTR(True)
		print "start connection with steer_motor"
	except:
		print "cannot start connection with steer_motor"
		sys.exit()

	time.sleep(2)
	rate = rospy.Rate(10)

	while  not rospy.is_shutdown():
		send_devision_to_steer_motor(ser)
		rate.sleep()

	ser.close()


def send_devision_to_steer_motor(ser):

	send_str = ""

	if 0 <= send_d and send_d < 10:
		send_str = "L0"+str(send_d)
	elif 10 <= send_d and send_d < 20:
		send_str = "L"+str(send_d)
	elif 20 <= send_d:
		send_str = "L20"
	elif -10 < send_d and send_d < 0:
		send_str = "R0"+str(-send_d)
	elif -20 < send_d and send_d <= -10:
		send_str = "R"+str(-send_d)
	elif send_d <= -20:
		send_str = "R20"

	rospy.loginfo(send_str)
	ser.write(send_str)
	

if __name__ == '__main__':
	rospy.init_node('steer_motor')
	rospy.Subscriber("input_devision", Int8, callback)
	connect_with_arduino()
