#!/usr/bin/env python
# coding: UTF-8

import rospy
from geometry_msgs.msg import Twist
from ultimate_seniorcar.msg import SeniorcarState
import numpy


PROCESSING_RATE = 50.0

if __name__ == '__main__':

	rospy.init_node('generate_seniorcar_command')
	pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)

	pub_command_array = []
	rate = rospy.Rate(PROCESSING_RATE)

	############################ read text file

	f=open('/home/ishikawa/state.txt','r')
	num = 0

	for i in f.readlines():
		i=i.strip()        #末尾の改行を除去
		i=i.split(",")     #カンマで文字列を区切り、リストを作成

		pub_command = SeniorcarState()

		pub_command.accel_opening = float(i[2])
		pub_command.max_velocity  = numpy.clip(float(i[3]),2.0,6.0)
		pub_command.steer_angle   = float(i[1])

		pub_command_array.append(pub_command)
		num += 1

	f.close()
	print "success to load %d state" % num

	############################################### 

	count = 0

	while count < num:
		pub.publish(pub_command_array[count])
		count += 1
		rate.sleep()

	passed_time = 0
