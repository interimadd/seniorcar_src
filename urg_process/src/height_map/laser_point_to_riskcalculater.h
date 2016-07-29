#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#include "nav_msgs/Odometry.h"
#include "risk_calculater.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <algorithm>


ros::Publisher heightmap_pub;
ros::Publisher result_pub;
tf::TransformListener* listener;

ros::Time start_time;

ultimate_seniorcar::SeniorcarState now_state;
nav_msgs::Odometry now_odom;

RiskCalculater calculater(0,0,200,200,0.03);

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
void SeniorcarStateCallback(const ultimate_seniorcar::SeniorcarState& msg);
void OdmetryCallback(const nav_msgs::Odometry& msg);
int main(int argc, char **argv);
