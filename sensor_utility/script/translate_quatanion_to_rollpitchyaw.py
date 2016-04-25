#!/usr/bin/env python  
# coding: UTF-8

#orentationをロールピッチヨーに変換してdegで表示

import rospy
import tf
import math

from nav_msgs.msg import Odometry
from numpy import *
from tf.transformations import euler_from_quaternion

RAD_TO_DEG = 180 / math.pi
roll  = 0
pitch = 0
yaw   = 0

def callback(last_odom):
    global roll
    global pitch
    global yaw
    (roll,pitch,yaw) = euler_from_quaternion([last_odom.pose.pose.orientation.x,last_odom.pose.pose.orientation.y,last_odom.pose.pose.orientation.z,last_odom.pose.pose.orientation.w])
    roll  = roll  * RAD_TO_DEG
    pitch = pitch * RAD_TO_DEG
    yaw   = yaw   * RAD_TO_DEG


if __name__ == '__main__':

    topic = rospy.get_param('odom_topic',"seniorcar_odometry")
    rospy.init_node('translate_q_to_deg')
    rospy.Subscriber(topic, Odometry, callback)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        print "roll:%f,pitch:%f,yaw:%f" % (roll,pitch,yaw)
        rate.sleep()
