#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
import tf
from ultimate_seniorcar.msg import SeniorcarState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu

CALCULATE_3DIMENTION_ODOMETRY = True  # 三次元的な動きを計算するかどうか ロールピッチヨーは車両ではなくワールド座標に固定されてるっぽい？参考url http://www.buildinsider.net/small/bookkinectv2/0804
STEER_ANGLE_OFFSET = 0.0

COV = 0.0005356910249999999
COV_MATRIX = [1e-3, 0.0, 0.0, 0.0, 0.0, 0.0, 
              0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, 
              0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 
              0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 
              0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 
              0.0, 0.0, 0.0, 0.0, 0.0, 1e3]
WHEEL_BASE = 0.9 # 車体のホイールベース
#RIGHT_TURN_MAGNIFICATION = 1.04267 # 右旋回時に実際の値より1.05倍程度になることから補正
#LEFT_TURN_MAGNIFICATION  = 0.95674 # 左旋回時に実際の値より0.95倍程度になることから補正
RIGHT_TURN_MAGNIFICATION = 1.1
LEFT_TURN_MAGNIFICATION = 1.1

IMU_PITCH_DEFFULT_ANGLE = -1.0
IMU_ROLL_DEFAULT_ANGLE  = 2.0


class OdometryCalculator:

    odometry = Odometry()
    seniorcar_command = SeniorcarState()
    t = 0
    imu_pitch = 0
    imu_roll = 0

    def __init__(self):

        rospy.init_node('seniorcar_odometry', anonymous=True)
        rospy.Subscriber("seniorcar_state", SeniorcarState, self.update_odometry)
        rospy.Subscriber('imu/data', Imu, self.update_pitch_roll)
        self.pub = rospy.Publisher('seniorcar_odometry',Odometry, queue_size=1000)

        self.current_time = rospy.get_rostime()
        self.last_time = rospy.get_rostime()

        self.odometry.header.frame_id = "odom"
        self.odometry.child_frame_id  = "base_link"
        self.odometry.pose.covariance  = COV_MATRIX
        self.odometry.twist.covariance = COV_MATRIX
        self.odometry.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))

        global CALCULATE_3DIMENTION_ODOMETRY
        global STEER_ANGLE_OFFSET
        global IMU_PITCH_DEFFULT_ANGLE
        global IMU_ROLL_DEFAULT_ANGLE
        CALCULATE_3DIMENTION_ODOMETRY = rospy.get_param('~calculate_3dimention_odmetry',CALCULATE_3DIMENTION_ODOMETRY)
        STEER_ANGLE_OFFSET = rospy.get_param('~steer_angle_offset',STEER_ANGLE_OFFSET)
        IMU_PITCH_DEFFULT_ANGLE = rospy.get_param('~imu_pitch_offset',IMU_PITCH_DEFFULT_ANGLE) * math.pi / 180.0
        IMU_ROLL_DEFAULT_ANGLE= rospy.get_param('~imu_roll_offset',IMU_ROLL_DEFAULT_ANGLE) * math.pi / 180.0


    def update_odometry(self,data):

        self.current_time = rospy.get_rostime()
        dt = self.current_time.to_sec() - self.last_time.to_sec()
        self.odometry.header.stamp = self.current_time

        if data.direction_switch == 0:
            data.vehicle_velocity = -1.0 * data.vehicle_velocity

        data.steer_angle += STEER_ANGLE_OFFSET
        #if abs(data.steer_angle) < 0.0:
        #    data.steer_angle = 0
 
        v = data.vehicle_velocity
        w = data.vehicle_velocity * math.tan(data.steer_angle*math.pi/180.0) / WHEEL_BASE

        if data.steer_angle > 0:
            w = w / LEFT_TURN_MAGNIFICATION
        else:
            w = w / RIGHT_TURN_MAGNIFICATION

        deltaTheta = w * dt

        last_odom = self.odometry
        (roll,pitch,yaw) = euler_from_quaternion([last_odom.pose.pose.orientation.x,last_odom.pose.pose.orientation.y,last_odom.pose.pose.orientation.z,last_odom.pose.pose.orientation.w])

        if CALCULATE_3DIMENTION_ODOMETRY:

            self.odometry.pose.pose.position.x += v * dt * math.cos(yaw + deltaTheta/2.0) * math.cos(self.imu_pitch) 
            self.odometry.pose.pose.position.y += v * dt * math.sin(yaw + deltaTheta/2.0) * math.cos(self.imu_pitch) 
            self.odometry.pose.pose.position.z -= v * dt * math.sin(self.imu_pitch) 
            self.odometry.twist.twist.linear.x  = v * math.cos(yaw) 
            self.odometry.twist.twist.linear.y  = v * math.sin(yaw) 
            self.odometry.twist.twist.linear.z  = v * ( math.sin(yaw + deltaTheta/2.0) * math.sin(self.imu_roll) - math.cos(self.imu_roll) * math.sin(self.imu_pitch) * math.cos(yaw + deltaTheta/2.0) )
            self.odometry.twist.twist.angular.z = w
            self.odometry.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(self.imu_roll , self.imu_pitch, yaw + deltaTheta))

        else:

            self.odometry.pose.pose.position.x += v * dt * math.cos(yaw + deltaTheta/2.0)
            self.odometry.pose.pose.position.y += v * dt * math.sin(yaw + deltaTheta/2.0)
            self.odometry.twist.twist.linear.x  = v * math.cos(yaw) 
            self.odometry.twist.twist.linear.y  = v * math.sin(yaw) 
            self.odometry.twist.twist.angular.z = w
            self.odometry.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0 , 0, yaw + deltaTheta))

        self.last_time = rospy.get_rostime()
        self.t = self.t + dt


    def update_pitch_roll(self,data):

        (self.imu_roll,self.imu_pitch,yaw) = euler_from_quaternion([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])        
        self.imu_roll += math.pi + IMU_ROLL_DEFAULT_ANGLE  # rollの処理謎
        self.imu_pitch -= IMU_PITCH_DEFFULT_ANGLE
        self.imu_pitch *= -1
        """
        self.imu_yaw   =  math.atan2(2.0 * data.orientation.x * data.orientation.y + 2.0 * data.orientation.w * data.orientation.z , data.orientation.w * data.orientation.w + data.orientation.x * data.orientation.x - data.orientation.y * data.orientation.y - data.orientation.z * data.orientation.z)
        self.imu_pitch =  -math.asin( 2.0 * data.orientation.w * data.orientation.y - 2.0 * data.orientation.x * data.orientation.z)
        self.imu_roll  =  math.atan2(2.0 * data.orientation.y * data.orientation.z + 2.0 * data.orientation.w * data.orientation.x, -data.orientation.w * data.orientation.w + data.orientation.x * data.orientation.x + data.orientation.y * data.orientation.y - data.orientation.z * data.orientation.z)
        """

    def publish_loop(self):

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.pub.publish(self.odometry)
            rate.sleep()


if __name__ == '__main__':
    
    calculator = OdometryCalculator()
    calculator.publish_loop()