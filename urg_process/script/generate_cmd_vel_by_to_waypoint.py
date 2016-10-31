#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
import actionlib

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import *
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion

APPRORCH_THRESHOLD = 3.0    # 何mまで近づいたら次の点を指示するか
P_GAIN = 1.0                # 偏差に対してどれだけの目標角速度値を出力するか
TARGET_VEL = 0.8            # デフォルトの直進速度値

class playRecordedWaypoint:

    recorded_waypoints = []
    index = 0
    pose_info = Pose()
    send_cmd_vel = Twist()

    def __init__(self):
        rospy.init_node('play_waypoint', anonymous=True)
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.get_waypoints_from_txt()
        odom_topic = rospy.get_param('odom_topic',"/amcl_pose")
        rospy.Subscriber(odom_topic, PoseWithCovarianceStamped, self.callback)

    def calc_distance_between_poses(self,pose1,pose2):
        return math.sqrt( abs( (pose1.position.x - pose2.position.x)*(pose1.position.x - pose2.position.x) + (pose1.position.y - pose2.position.y)*(pose1.position.y - pose2.position.y) ) )

    # 自車両位置情報と目標通過点の更新
    def callback(self,data):
        self.pose_info = data.pose.pose
    
        if len(self.recorded_waypoints) - 1 > self.index:
            while self.calc_distance_between_poses( self.recorded_waypoints[self.index] , self.pose_info ) < APPRORCH_THRESHOLD:
                self.index += 1
                if len(self.recorded_waypoints) - 1 <= self.index:
                    self.index = 0
                    while self.calc_distance_between_poses( self.recorded_waypoints[self.index] , self.pose_info ) < APPRORCH_THRESHOLD:
                        self.index += 1
                    break

    def get_waypoints_from_txt(self):

        filename = rospy.get_param('waypoint_file_path',"/media/ishikawa/DATAPART1/HDD/bagfiles/kashiwa_campus/long_guruguru_waypoint.txt")
        print "open " + str(filename)

        f=open(filename,'r')
        num = 0

        for i in f.readlines():
        
            #print i
            i=i.strip()         #末尾の改行を除去
            i=i.split("\t")     #空白(tab)で文字列を区切り、リストを作成

            pose_info = Pose()

            pose_info.position.x    = float(i[0])
            pose_info.position.y    = float(i[1])
            pose_info.position.z    = float(i[2])
            pose_info.orientation.x = float(i[3])
            pose_info.orientation.y = float(i[4])
            pose_info.orientation.z = float(i[5])
            pose_info.orientation.w = float(i[6])

            self.recorded_waypoints.append(pose_info)
            num += 1

            print pose_info

        f.close()
        print "success to load %d waypoints" % len(self.recorded_waypoints)

    # 目標速度と角速度の計算
    # 角速度は、自車両から見た目標通過点のヨー角度にゲインをかけて計算
    def updata_cmd_vel(self):

        q = self.pose_info.orientation
        (roll,pitch,vehicle_yaw) = euler_from_quaternion([q.x,q.y,q.z,q.w])

        map_yaw = math.atan2( self.recorded_waypoints[self.index].position.y - self.pose_info.position.y , self.recorded_waypoints[self.index].position.x - self.pose_info.position.x)
        div_yaw = map_yaw - vehicle_yaw
        div_yaw_mod = div_yaw

        # -PIと+PIの間が不連続に切り替わるため、この付近での整合性を持たせる処理
        if div_yaw > 2.0:
            div_yaw_mod -= 2 * math.pi
        elif div_yaw < -2.0:
            div_yaw_mod += 2 * math.pi

        print map_yaw,vehicle_yaw,div_yaw,div_yaw_mod,self.index

        self.send_cmd_vel.linear.x = TARGET_VEL
        self.send_cmd_vel.angular.z = div_yaw_mod * P_GAIN

    def calculate_and_publish_cmd_vel(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.updata_cmd_vel()
            self.pub.publish(self.send_cmd_vel)
            rate.sleep()


if __name__ == '__main__':
    processer = playRecordedWaypoint()
    processer.calculate_and_publish_cmd_vel()