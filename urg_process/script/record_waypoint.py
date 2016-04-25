#!/usr/bin/env python
# coding: UTF-8

import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

RECORD_THRESHOLD = 2.0  # 何mおきに記録するか

recorded_waypoints = [] 

def calc_distance_between_poses(pose1,pose2):
    return math.sqrt( abs( (pose1.position.x - pose2.position.x)*(pose1.position.x - pose2.position.x) + (pose1.position.y - pose2.position.y)*(pose1.position.y - pose2.position.y) ) )

def callback(data):

    pose_info = data.pose.pose
    
    if len(recorded_waypoints) == 0:
        recorded_waypoints.append(pose_info)
    elif calc_distance_between_poses(recorded_waypoints[-1],pose_info) > RECORD_THRESHOLD:
        recorded_waypoints.append(pose_info)

def output_to_txt():

    f=open('waypoint.txt','w')

    for i in range( len(recorded_waypoints) ):
        f.write( str(recorded_waypoints[i].position.x) )
        f.write("\t")
        f.write( str(recorded_waypoints[i].position.y) )
        f.write("\t")
        f.write( str(recorded_waypoints[i].position.z) )
        f.write("\t")
        f.write( str(recorded_waypoints[i].orientation.x) )
        f.write("\t")
        f.write( str(recorded_waypoints[i].orientation.y) )
        f.write("\t")
        f.write( str(recorded_waypoints[i].orientation.z) )
        f.write("\t")
        f.write( str(recorded_waypoints[i].orientation.w) )
        f.write("\n")

    f.close()


def listener():

    rospy.init_node('record_waypoint', anonymous=True)

    odom_topic = rospy.get_param('odom_topic',"/amcl_pose")

    rospy.Subscriber(odom_topic,PoseWithCovarianceStamped, callback)

    rospy.spin()

    output_to_txt()

if __name__ == '__main__':
    listener()