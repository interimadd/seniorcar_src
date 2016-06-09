#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
import sys
from std_msgs.msg import Int8
from std_msgs.msg import String

send_d = 0
MAX_DIF = 40
send_str = ""

def callback(data):
	global send_d
	send_d = data.data

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
		send_devision_to_steer_motor(ser)
	except:
		print "cannot start connection with steer_motor"
		sys.exit()

	time.sleep(2)
	rate = rospy.Rate(50)

	while  not rospy.is_shutdown():
		send_devision_to_steer_motor(ser)
		pub_str.data = send_str
		pub.publish(pub_str)
		rate.sleep()

	ser.close()


def send_devision_to_steer_motor(ser):

	global send_str

	if 0 <= send_d and send_d < 10:
		send_str = "L0"+str(send_d)
	elif 10 <= send_d and send_d < MAX_DIF:
		send_str = "L"+str(send_d)
	elif MAX_DIF <= send_d:
		send_str = "L"+str(MAX_DIF)
	elif -10 < send_d and send_d < 0:
		send_str = "R0"+str(-send_d)
	elif -MAX_DIF < send_d and send_d <= -10:
		send_str = "R"+str(-send_d)
	elif send_d <= -MAX_DIF:
		send_str = "R"+str(MAX_DIF)

	rospy.loginfo(send_str)
	ser.write(send_str)
	

if __name__ == '__main__':
	rospy.init_node('steer_motor')
	rospy.Subscriber("input_devision", Int8, callback)
	connect_with_arduino()
