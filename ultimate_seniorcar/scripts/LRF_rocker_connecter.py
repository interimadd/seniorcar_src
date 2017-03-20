#!/usr/bin/env python
# coding: UTF-8

# LRF揺動機構のモータのパルス取得とTF変換を行うプログラム

import rospy
import time
import serial
import math
import tf
from multiprocessing import Process, Value
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float64


PALUSE_NUM_PER_ROTATION = 1000.0 # 一回転あたりのパルス数
REDUCTION_RATIO = 100.0          # ギアの減速比
MOTOR_GEAR_NUM = 33.0
ROCKER_GEAR_NUM = 60.0

GEAR_CONSTANT =  2.0 * 3.14 / float(REDUCTION_RATIO * PALUSE_NUM_PER_ROTATION) * ( MOTOR_GEAR_NUM / ROCKER_GEAR_NUM) #パルス→回転角度計算用

def conecct_with_arduino_process(palse_count):

	port = rospy.get_param('rocker_encorder_port',"/dev/ttyACM0")
	try:
		ser = serial.Serial(port,timeout=0.05)
		ser.setDTR(False)
		time.sleep(1)
		ser.setDTR(True)
		print "start connection with rotarty encoder"
	except:
		print "cannot start connection with rotarty encoder"

	time.sleep(1)

	ser.write("s");

	while  not rospy.is_shutdown():
		try:
			line  = ser.readline().rstrip()
			if len(line) > 0:
				if line.isdigit():
					palse_count.value = int(line)
				elif line[0] == "-":
					if line[1:].isdigit():
						palse_count.value = int(line)
		except KeyboardInterrupt:
			print "Ctrl+C"
			break

	print "End Falg Send"
	ser.write("e");

	ser.close()


def calculate_odometory_process(palse_count):

	rospy.init_node('LRF_rocker_connecter', anonymous=True)
	br = tf.TransformBroadcaster()
	pub = rospy.Publisher('LRF_rocker_angle', Float64 , queue_size=10)
	rate = rospy.Rate(50.0)

	while not rospy.is_shutdown():
		try:
			pitch = float(palse_count.value) * GEAR_CONSTANT
			q = tf.transformations.quaternion_from_euler(0, pitch, 0)
			br.sendTransform((1.05, 0, 0.65),q,rospy.Time.now(),"LRF_rocker_base","base_link")
			pub.publish(pitch)
			rate.sleep()
			print palse_count.value,pitch
		except KeyboardInterrupt:
			print "End TF process"
			break


if __name__ == '__main__':
	palse_count = Value('d',0)
	connect_process = Process( target = conecct_with_arduino_process , args = (palse_count,) )
	calculate_process = Process( target = calculate_odometory_process, args = (palse_count,) )
	try:
		connect_process.start()
		calculate_process.start()
	except rospy.ROSInterruptException:
		pass