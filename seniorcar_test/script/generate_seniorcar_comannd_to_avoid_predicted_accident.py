#!/usr/bin/env python
# coding: UTF-8

import rospy
from geometry_msgs.msg import Twist
from ultimate_seniorcar.msg import AccidentPredictResult
from ultimate_seniorcar.msg import SeniorcarState

PUBLISH_RATE = 50.0       # コマンドをパブリッシュする周期

APPROACH_THRESHOLD = 1.0  # 何mまで危険な領域に直進で接近させるか
AVOID_THRESHOLD = 1.5     # 避けるとしたら何mまで余裕がある角度か

GO_STRAIGHT_TIME = 8.0    # 真っ直ぐに進む時間


class AvoidPredictedAccident:

	seniorcar_command = SeniorcarState()
	seniorcar_command.accel_opening = 0
	seniorcar_command.steer_angle = 0
	seniorcar_command.max_velocity = 2.0

	def __init__(self):
		rospy.init_node('generate_seniorca_comand_to_avoid_predicted_accident')
		self.pub = rospy.Publisher('seniorcar_command', SeniorcarState, queue_size=10)
	
	def subscribe_predicted_result(self):
		rospy.Subscriber("accident_predict", AccidentPredictResult , self.accidentsubCallback)

	def accidentsubCallback(self,msg):
		data_num = len(msg.steer_angle)
		steer0deg_index = data_num/2 # 操舵角度0度の番号

		if msg.max_distance[steer0deg_index] > APPROACH_THRESHOLD:
			# 前方1mが安全なら直進
			self.seniorcar_command.accel_opening = 120
			self.seniorcar_command.steer_angle   = 0
		else:
			# 1.5m先まで走行できる経路の中で0degに近い場所を探す。そもそも無ければ停止する
			avoid_index = -1
			for i in range(0,data_num):
				if msg.max_distance[i] > AVOID_THRESHOLD:
					if abs( steer0deg_index - i ) < abs( steer0deg_index - avoid_index ):
						avoid_index = i
			if avoid_index == -1:
				self.seniorcar_command.accel_opening = 0
				self.seniorcar_command.steer_angle   = 0
			else:
				self.seniorcar_command.accel_opening = 120
				self.seniorcar_command.steer_angle   = msg.steer_angle[avoid_index] * 180.0 /3.14 

	def comannd_to_go_straight(self):
		count = 0
		rate = rospy.Rate(PUBLISH_RATE)
		while not rospy.is_shutdown() and count < GO_STRAIGHT_TIME * PUBLISH_RATE:
			self.seniorcar_command.accel_opening = 120
			self.seniorcar_command.steer_angle = 0
			self.pub.publish(self.seniorcar_command)
			count += 1
			rate.sleep()

	def calculate_and_publish_command(self):
		rate = rospy.Rate(PUBLISH_RATE)
		while not rospy.is_shutdown():
			self.pub.publish(self.seniorcar_command)
			rate.sleep()

if __name__ == '__main__':
	calclater = AvoidPredictedAccident()
	calclater.subscribe_predicted_result()
	calclater.comannd_to_go_straight()
	calclater.calculate_and_publish_command()
