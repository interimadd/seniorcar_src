#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
import sys
from std_msgs.msg import Int8
from std_msgs.msg import String

send_d = 0
MAX_DIF = 15
send_str = ""

MAX_ANGLE = 40
GERA_CONSTANT = 3000.0 * 160.0 * 1.561 / 360.0


def callback(data):
	global send_d
	send_d = -data.data

def connect_with_arduino():
	global send_str
	port = rospy.get_param('steer_motor_port',"/dev/ttyACM0")
	pub = rospy.Publisher('arduino_input', String, queue_size=10)
	pub_str = String()
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
	rate = rospy.Rate(4)
	send_devision_to_steer_motor(ser)

	while  not rospy.is_shutdown():
		serial_bit = ser.readline()
		if "OK" in serial_bit:
			pub_str.data = send_str
			pub.publish(pub_str)
			send_devision_to_steer_motor(ser)
			print serial_bit
		rate.sleep()

	ser.close()


def send_devision_to_steer_motor(ser):

	global send_str

	target_value = max( -MAX_ANGLE , min( send_d , MAX_ANGLE ) ) * GERA_CONSTANT
	send_str = "LR" + str(int(target_value)) + "\n"
	rospy.loginfo(send_str)
	ser.write(send_str)
	ser.write("M\n")
	

if __name__ == '__main__':
	rospy.init_node('steer_motor')
	rospy.Subscriber("input_devision", Int8, callback)
	connect_with_arduino()
