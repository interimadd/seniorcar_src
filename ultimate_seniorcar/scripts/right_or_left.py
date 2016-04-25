#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from sensor_msgs.msg import LaserScan
from ultimate_seniorcar.msg import SeniorcarState

class Right_Left:

    seniorcar_command = SeniorcarState()

    def __init__(self):

        rospy.init_node('right_or_left', anonymous=True)
        rospy.Subscriber("scan", LaserScan, self.callback)
        self.pub = rospy.Publisher('seniorcar_command',SeniorcarState, queue_size=1000)
        self.pub.publish(self.seniorcar_command)


    def callback(self,data):

        data_num = len(data.ranges)
        r_count = 0
        l_count = 0

        #LRF極座標データを直行座標データに変換(x,y)
        for i in range(0,data_num-1):
            if data.ranges[i] < 4.0 and data.ranges[i] > 0.2 :
                x = data.ranges[i] * math.sin( data.angle_min + data.angle_increment * i )
                y = data.ranges[i] * math.cos( data.angle_min + data.angle_increment * i )
                if 0.0 < y and y < 2.0:
                    if 0.0 < x and x < 1.0:
                        l_count += 1
                    elif -1.0 < x and x < 0:
                        r_count += 1

        print r_count,l_count

        if r_count > 3:
            self.seniorcar_command.right_winker = 1
        else:
            self.seniorcar_command.right_winker = 0

        if l_count > 3:
            self.seniorcar_command.left_winker  = 1
        else:
            self.seniorcar_command.left_winker  = 0

                
    def publish_loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.seniorcar_command)
            rate.sleep()


if __name__ == '__main__':
    
    right_left = Right_Left()
    right_left.publish_loop()    