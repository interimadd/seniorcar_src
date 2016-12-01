#!/usr/bin/env python  
# coding: UTF-8

#orentationをロールピッチヨーに変換してdegで表示

import rospy
import tf
import math

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from numpy import *
from tf.transformations import euler_from_quaternion

RAD_TO_DEG = 180.0 / math.pi
roll  = 0
pitch = 0
yaw   = 0

now_odom = Odometry()

def callback(data):

    q = Quaternion()
    q = data.orientation 
    global roll
    global pitch
    global yaw

    (roll,pitch,yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])
    #yaw   =  math.atan2(2.0 * q.x * q.y + 2.0 * q.w * q.z , q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z)
    #pitch =  math.asin( 2.0 * q.w * q.y - 2.0 * q.x * q.z)
    #roll  =  math.atan2(2.0 * q.y * q.z + 2.0 * q.w * q.x, -q.w * q.w + q.x * q.x + q.y * q.y - q.z * q.z)

    roll  = roll  * RAD_TO_DEG
    pitch = pitch * RAD_TO_DEG
    yaw   = yaw   * RAD_TO_DEG


def odomcallback(data):

    global now_odom
    now_odom = data


if __name__ == '__main__':

    rospy.init_node('translate_q_to_deg')
    rospy.Subscriber('imu/data', Imu, callback)
    rospy.Subscriber('seniorcar_odometry', Odometry, odomcallback)

    rate = rospy.Rate(20.0)

    print "time,pos_x,pos_y,roll,pitch,yaw"

    while not rospy.is_shutdown():
        now = rospy.get_rostime()
        print "%d.%09d,%f,%f,%f,%f,%f" % ( now.secs,now.nsecs,now_odom.pose.pose.position.x,now_odom.pose.pose.position.y,roll,pitch,yaw)
        rate.sleep()
