#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#include "elevation_map/elevation_map.h"
#include "risk_calculater_for_elevation_map/risk_calculater_for_elevation_map.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <algorithm>
#include "nav_msgs/Odometry.h"


ros::Publisher heightmap_pub;
ros::Publisher result_pub,pose_pub;

ros::Publisher road_pub;
ros::Publisher grass_pub;
ros::Publisher others_pub;

tf::TransformListener* listener;

ros::Time start_time;

ultimate_seniorcar::SeniorcarState now_state;
nav_msgs::Odometry now_odom;

RiskCalculater elevation_map(0,0,100,100,0.05);

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
void SeniorcarStateCallback(const ultimate_seniorcar::SeniorcarState& msg);
void OdmetryCallback(const nav_msgs::Odometry& msg);
int main(int argc, char **argv);
