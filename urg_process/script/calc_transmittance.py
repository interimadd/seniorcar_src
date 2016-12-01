#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

RANGE_RESOLUTION = 0.01
MIN_RANGE = 0.1
MAX_RANGE = 3.0

PENETRATE = 0
REFLECT   = 1


class CalcTransmittance:

    transmittance_datas = [] #[角度][距離][通過/反射]
    initialize_flag = False
    data_num = 0
    data_incliment = 0.0
    data_angle_min = 0.0

    def __init__(self):

        rospy.init_node('calc_transmittance', anonymous=True)
        rospy.Subscriber("scan_tilt", LaserScan, self.callback)

    def callback(self,data):

        if self.initialize_flag == False:
            self.initializeTransmittanceDatas(data)
            self.initialize_flag = True

        for i in range(0,self.data_num-1):
            if data.ranges[i] < MAX_RANGE and data.ranges[i] > MIN_RANGE  :
                distance_index = int ( (data.ranges[i] - MIN_RANGE) / RANGE_RESOLUTION )
                if distance_index < (MAX_RANGE-MIN_RANGE)/RANGE_RESOLUTION:
                    for j in range(0,distance_index):
                        self.transmittance_datas[i][j][0] += 1
                    self.transmittance_datas[i][distance_index][1] += 1

    def initializeTransmittanceDatas(self,data):

        self.data_num = len(data.ranges)
        self.data_angle_min = data.angle_min
        self.data_incliment = data.angle_increment

        for i in range(0,self.data_num-1):
            tmp_data_array = []
            for j in range(0,int((MAX_RANGE-MIN_RANGE)/RANGE_RESOLUTION)):
                tmp_data_array.append([0.0,0.0])
            self.transmittance_datas.append(tmp_data_array)


    def printTransmittanceDatas(self):

        #print self.transmittance_datas
        """
        for i in range(0,self.data_num-1):
            print "ang:" + str( self.data_angle_min + self.data_incliment * i),
            for j in range(0,int((MAX_RANGE-MIN_RANGE)/RANGE_RESOLUTION)):
                print "(" + str(self.transmittance_datas[i][j][0]) + "," + str(self.transmittance_datas[i][j][1]) + ")",
            print ""
        """

        fig = plt.figure()
        ax = fig.add_subplot(1,1,1)

        ax.set_title('transmittance plot')
        ax.set_xlabel('x')
        ax.set_ylabel('y')

        """
        x = [1]
        y = [2]

        ax.scatter(x,y)

        x = [3]
        y = [4]

        ax.scatter(x,y)
        """
        x_pos = []
        y_pos = []
        value = []
        for i in range(0,self.data_num-1):
            for j in range(0,int((MAX_RANGE-MIN_RANGE)/RANGE_RESOLUTION)):
                if self.transmittance_datas[i][j][0] > 0:
                    angle = self.data_angle_min + self.data_incliment * float(i) #  - 0.15
                    x_pos.append(( float(j) * RANGE_RESOLUTION ) * math.sin(angle))
                    y_pos.append(( float(j) * RANGE_RESOLUTION ) * math.cos(angle))
                    #value.append(self.transmittance_datas[i][j][0]+self.transmittance_datas[i][j][1])
                    value.append(float(self.transmittance_datas[i][j][1]) / float((self.transmittance_datas[i][j][0]+self.transmittance_datas[i][j][1])))
                """
                if self.transmittance_datas[i][j][0] > 0:
                    color = 1.0
                    ax.scatter(x_pos,y_pos)
                else:
                    ax.scatter(x_pos,y_pos) 
                """

        #print value
        ax.scatter(x_pos,y_pos, s=6, c=value,linewidths=0, alpha=0.5,cmap='Blues',vmin=0.0, vmax=1.0)
        #ax.scatter(x_pos,y_pos, s=6, c=value,linewidths=0, alpha=0.5,cmap='Blues',vmin=0.0, vmax=200)
        fig.show()


if __name__ == '__main__':
    
    calculator = CalcTransmittance()
    raw_input(">>> print")
    calculator.printTransmittanceDatas()
    raw_input(">>> end")
    #rospy.spin()