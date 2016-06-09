#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import serial
import sys
from std_msgs.msg import Int8
from std_msgs.msg import String
from multiprocessing import Process, Value, Array, Queue


MAX_DIF = 50

class SteerMotorArduinoConnecter:

	nantoka_str = String()


	def __init__(self):

		rospy.init_node('steer_motor')
		rospy.Subscriber("input_devision", Int8, self.callback)

		r = Queue()

		self.pub_send   = rospy.Publisher('arduino_input', String, queue_size=10)
		self.pub_recive = rospy.Publisher('arduino_recive', String, queue_size=10)

		self.send_d = Value('i',0)

		self.connect_with_arduino()

		self.read_process  = Process( target = self.recive_arduino, args=(r,))

		try:
			self.read_process.start()
		except:
			print "process cannot start"
			sys.exit()

		rval = String()

		rate = rospy.Rate(100)
		pub_str=String()

		while not rospy.is_shutdown():

			if 0 <= self.send_d and self.send_d < 10:
				send_str = "L0"+str(self.send_d)
			elif 10 <= self.send_d and self.send_d < MAX_DIF:
				send_str = "L"+str(self.send_d)
			elif MAX_DIF <= self.send_d:
				send_str = "L"+str(MAX_DIF)
			elif -10 < self.send_d and self.send_d < 0:
				send_str = "R0"+str(-self.send_d)
			elif -MAX_DIF < self.send_d and self.send_d <= -10:
				send_str = "R"+str(-self.send_d)
			elif self.send_d <= -MAX_DIF:
				send_str = "R"+str(MAX_DIF)

			self.ser.write(send_str)
			pub_str.data = send_str
			#rospy.loginfo(pub_str)
			self.pub_send.publish(pub_str)
			rate.sleep()

			try:	
				rval = r.get(True,0.001)
				self.pub_recive.publish(rval)
			except:
				True

		self.ser.close()


	def connect_with_arduino(self):

		port = rospy.get_param('steer_motor_port',"/dev/ttyACM0")
		print port

		# シリアル通信の開始
		try:
			self.ser = serial.Serial(port,9600)
			self.ser.setDTR(False)
			time.sleep(1)
			self.ser.setDTR(True)
			print "start connection with steer_motor"
	
		except:
			print "cannot start connection with steer_motor"
			sys.exit()

		time.sleep(1)
	

	def callback(self,data):

		self.send_d = data.data


	def recive_arduino(self,r):

		recive_str = ""
		tmp_str = ""
		pub_str = String()
		i = 0

		while  not rospy.is_shutdown():
			tmp_str = self.ser.read()
			if tmp_str == ",":
				pub_str.data = recive_str
				#self.pub_recive.publish(pub_str)
				r.put(pub_str)
				recive_str = ""
				i = 0
			else:
				recive_str += tmp_str
			i += 1
	

if __name__ == '__main__':
	connecter = SteerMotorArduinoConnecter()
