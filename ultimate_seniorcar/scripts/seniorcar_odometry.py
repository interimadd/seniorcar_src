#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
import tf
from ultimate_seniorcar.msg import SeniorcarState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion


COV = 0.0005356910249999999
COV_MATRIX = [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
              0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 
              0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 
              0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 
              0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 
              0.0, 0.0, 0.0, 0.0, 0.0, 1e3]
WHEEL_BASE = 0.9 # 車体のホイールベース
STEEA_ANGLE_OFFSET = 1.0 # セニアカーのCAN情報を直接読み取ると何故か直進時に-1.0deg程度になるので、それを補正する用
RIGHT_TURN_MAGNIFICATION = 1.04267 # 右旋回時に実際の値より1.05倍程度になることから補正
LEFT_TURN_MAGNIFICATION  = 0.95674 # 左旋回時に実際の値より0.95倍程度になることから補正


class OdometryCalculator:

    odometry = Odometry()
    seniorcar_command = SeniorcarState()
    t = 0

    def __init__(self):

        rospy.init_node('seniorcar_odometry', anonymous=True)
        rospy.Subscriber("seniorcar_state", SeniorcarState, self.update_odometry)
        self.pub = rospy.Publisher('seniorcar_odometry',Odometry, queue_size=1000)

        self.current_time = rospy.get_rostime()
        self.last_time = rospy.get_rostime()

        self.odometry.header.frame_id = "odom"
        self.odometry.child_frame_id  = "base_link"
        self.odometry.pose.covariance  = COV_MATRIX
        self.odometry.twist.covariance = COV_MATRIX
        self.odometry.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))


    def update_odometry(self,data):

        self.current_time = rospy.get_rostime()
        dt = self.current_time.to_sec() - self.last_time.to_sec()
        self.odometry.header.stamp = self.current_time

        if data.direction_switch == 0:
            data.vehicle_velocity = -1.0 * data.vehicle_velocity

        data.steer_angle += STEEA_ANGLE_OFFSET
        if abs(data.steer_angle) < 2.0:
            data.steer_angle = 0
 
        v = data.vehicle_velocity
        w = data.vehicle_velocity * math.tan(data.steer_angle*math.pi/180.0) / WHEEL_BASE

        if data.steer_angle > 0:
            w = w / LEFT_TURN_MAGNIFICATION
        else:
            w = w / RIGHT_TURN_MAGNIFICATION

        deltaTheta = w * dt

        last_odom = self.odometry
        (roll,pitch,yaw) = euler_from_quaternion([last_odom.pose.pose.orientation.x,last_odom.pose.pose.orientation.y,last_odom.pose.pose.orientation.z,last_odom.pose.pose.orientation.w])
        print yaw

        self.odometry.pose.pose.position.x += v * dt * math.cos(yaw + deltaTheta/2.0)
        self.odometry.pose.pose.position.y += v * dt * math.sin(yaw + deltaTheta/2.0)
        self.odometry.twist.twist.linear.x  = v * math.cos(yaw) 
        self.odometry.twist.twist.linear.y  = v * math.sin(yaw) 
        self.odometry.twist.twist.angular.z = w
        self.odometry.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw + deltaTheta))

        self.last_time = rospy.get_rostime()
        self.t = self.t + dt
        

    def publish_loop(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub.publish(self.odometry)
            rate.sleep()


if __name__ == '__main__':
    
    calculator = OdometryCalculator()
    calculator.publish_loop()