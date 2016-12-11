#!/usr/bin/env python
# coding: UTF-8

import rospy
import time
import numpy
import serial
import sys
from numpy import *
from multiprocessing import Process, Value, Array
from ultimate_seniorcar.msg import SeniorcarState

class CANUSB_Connecter:

	seniorcar_state = SeniorcarState()
	STEER_ANGLE_OFFSET = 0.0

	def __init__(self):
		rospy.init_node('canusb_connecter', anonymous=True)
		self.pub = rospy.Publisher('seniorcar_state',SeniorcarState, queue_size=1000)
		time.sleep(1)
		self.connect_with_canusb()
		self.pub.publish(self.seniorcar_state)
		print self.pub
		self.STEER_ANGLE_OFFSET = rospy.get_param('~steer_angle_offset',self.STEER_ANGLE_OFFSET) 


	def connect_with_canusb(self):

		# シリアル通信開始
		port = rospy.get_param('canusb_port',"/dev/ttyUSB1")
		print port
		try:
			self.ser = serial.Serial(port,9600)
			self.ser.setDTR(False)
			time.sleep(0.5)
			self.ser.setDTR(True)
			print "start connection with canusb"
		except:
			print "cannot start connection with canusb"
			sys.exit()

		# CANUSBのCANポートを開くコマンド
		# 改行コマンド×複数回→通信速度設定→ポートオープン
		time.sleep(0.1)
		self.ser.write("\r")
		self.ser.write("\r")
		time.sleep(0.1)
		self.ser.write("C\r")
		time.sleep(0.1)
		self.ser.write("S6\r")
		time.sleep(0.1)
		self.ser.write("O\r")

		print "can open"


	def write_candata(self):

		#A0への信号は0byteの場所にインクリメント(毎回変わる値)を入れてあげる必要がある
		#A0の6byteと7byteは同じにする
		#t0A0 2 22 33は機器構成認識信号（22 33 の部分は00 00 意外であればなんでもよい）
		#t0A1 8 00 00 accel 00 00 00 00 00 はアクセル開度の上書き
		#t0A5 8 incliment 01 00 00 direction_switch 00 00 1C other_commnad はアクセル開度以外の上書き.1Cは各信号を操作できるようにするしないのフラグ
		#other_commandの中身は2byteの長さで 0 0 0 hone r_winker l_winker 0 light

		#スズキからもらった資料の一部が誤り
		#外部最高速度はID:0A1の方
		
		incliment = 11
		wait_time = 0

		while  not rospy.is_shutdown():

			if self.can_override_flag.value == 0:
				self.ser.write("t0A022233\r")
				self.ser.write("t0A58"+str(incliment)+"00000000000000\r")
			elif self.can_override_flag.value == 1:
				self.ser.write("t0A022233\r")
				print self.command_message(incliment)
				self.ser.write(self.command_message(incliment))
				#self.ser.write("t0A58"+str(incliment)+"01000000000F0F\r")
				if wait_time < 1:
					wait_time = wait_time + 0.05
				else:
					self.ser.write(self.accel_command_message())
					0
			time.sleep(0.05)

			incliment = incliment + 1
			if incliment > 90:
				incliment = 11


	# CANUSBのアクセル開度の上書きコマンド文字列
	def accel_command_message(self):

		command = "t0A180000"
		command += '%02x' %  int(self.seniorcar_command_array[0])
		command += "00"
		#command += "00"
		command += '%02x' % int( (self.seniorcar_command_array[1] - 2.0)*25.0 )	#max_vel
		command += "000000"
		command += "\r"

		return command


	# CANUSBに送るシリアルコマンド文字列を生成
	def command_message(self,incliment):

		command = ""
		command += "t0A5" 
		command += "8"
		command += str(incliment)
		command += "010000"
		command += '%02x' % int( self.seniorcar_command_array[4] ) # direction_switch
		command += "001C"

		# 7byte目に格納する文字列の2進数表記
		hone   = int(self.seniorcar_command_array[8])
		r_wink = int(self.seniorcar_command_array[7])
		l_wink = int(self.seniorcar_command_array[6])
		light  = int(self.seniorcar_command_array[5])
		other_commands = "000"+str(hone)+str(r_wink)+str(l_wink)+"0"+str(light)
		other_commands = '%x' % int( other_commands, 2)
		
		if r_wink == 1:
			other_commands = "08"
		elif l_wink == 1:
			other_commands = "04"
		else:
			other_commands = "00"

		command += other_commands
		command += "\r"

		return command


	# 受信と送信にプロセスを分けて処理
	def start_spin(self):

		self.can_override_flag = Value('i',0)
		self.seniorcar_state_array   = Array('d',(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
		self.seniorcar_command_array = Array('d',(0.0,2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0))
		self.read_process  = Process( target = self.read_candata, args=(''))
		self.write_process = Process( target = self.write_candata,args=(''))

		try:
			self.read_process.start()
			self.write_process.start()
		except rospy.ROSInterruptException:
			pass

		rospy.Subscriber("seniorcar_command", SeniorcarState, self.command_recive)

		rate = rospy.Rate(100)
		while not rospy.is_shutdown():
			self.update_vehicle_data()
			self.pub.publish(self.seniorcar_state)
			rate.sleep()


	def command_recive(self,data):
		
		self.seniorcar_command_array[0] = numpy.clip(data.accel_opening, 0.0, 127)
		self.seniorcar_command_array[1] = numpy.clip(data.max_velocity , 2.0, 6.0)
		self.seniorcar_command_array[2] = data.steer_angle
		self.seniorcar_command_array[3] = data.vehicle_velocity * 3.6
		self.seniorcar_command_array[4] = data.direction_switch
		self.seniorcar_command_array[5] = data.light_signal
		self.seniorcar_command_array[6] = data.left_winker
		self.seniorcar_command_array[7] = data.right_winker
		self.seniorcar_command_array[8] = data.hone_signal


	def update_vehicle_data(self):

		self.seniorcar_state.accel_opening = self.seniorcar_state_array[0]
		self.seniorcar_state.max_velocity = self.seniorcar_state_array[1]
		self.seniorcar_state.steer_angle = self.seniorcar_state_array[2]
		self.seniorcar_state.vehicle_velocity = self.seniorcar_state_array[3] / 3.6  #  [km/h]から[m/s]へ変換
		self.seniorcar_state.direction_switch  = self.seniorcar_state_array[4]
		self.seniorcar_state.light_signal = self.seniorcar_state_array[5]
		self.seniorcar_state.left_winker = self.seniorcar_state_array[6]
		self.seniorcar_state.right_winker = self.seniorcar_state_array[7]
		self.seniorcar_state.hone_signal = self.seniorcar_state_array[8]

	
	def read_candata(self):

		line = ""
		can_id   = ""
		can_data = ""

		i = 0

		while  not rospy.is_shutdown():
			serial_bit = self.ser.read()
			if serial_bit == "\r" and i > 10:
				can_id   = line[:4]
				#can_bit  = line[4]
				can_data = line[5:]
				line = ""
				i = 0
				if can_id == "t01E":	# セニアカー走行データのID
					self.update_can_information_array(can_data)
				elif can_id == "t010":		# システム認識終了フラグの受信
					if can_data[6:8] == "84":
						if self.can_override_flag.value == 0:
							print "change to interupt mode"
						self.can_override_flag.value = 1
					else:
						if self.can_override_flag.value == 1:
							print "change to manual mode"
						self.can_override_flag.value = 0
				elif can_id == "t0A0":
					print can_data
			else:
				line = line + serial_bit
				i = i + 1

		self.ser.close()


	def update_can_information_array(self , can_data):

		self.seniorcar_state_array[0] = float(int(can_data[:2] , 16)) / 127.0 # アクセル開度
		self.seniorcar_state_array[1] = float(int(can_data[2:4] , 16)) / 100.0 * 4.0 + 2.0 #  最大速度
		self.seniorcar_state_array[2] = float(int(can_data[6:8]   + can_data[4:6]  , 16)) / 10.0 - 90.0 # 操舵角度
		self.seniorcar_state_array[3] = float(int(can_data[10:12] + can_data[8:10] , 16)) / 100.0 # 車両速度

		# 1byteのデータからbit情報を抽出
		byte7_data = int( can_data[14:16] ,16 )
		self.seniorcar_state_array[4] = byte7_data & 1
		self.seniorcar_state_array[5] = (byte7_data >> 1) & 1
		self.seniorcar_state_array[6] = (byte7_data >> 2) & 1
		self.seniorcar_state_array[7] = (byte7_data >> 3) & 1
		self.seniorcar_state_array[8] = (byte7_data >> 4) & 1

		self.seniorcar_state_array[2] -= self.STEER_ANGLE_OFFSET


if __name__ == '__main__':

	connecter = CANUSB_Connecter()
	connecter.start_spin()