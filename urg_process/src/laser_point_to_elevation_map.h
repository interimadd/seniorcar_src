#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#include "elevation_map/elevation_map.h"
#include "risk_calculater_for_elevation_map/risk_calculater_for_elevation_map.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <algorithm>


ros::Publisher heightmap_pub;
ros::Publisher result_pub;

ros::Publisher road_pub;
ros::Publisher grass_pub;
ros::Publisher others_pub;

tf::TransformListener* listener;
RiskCalculater elevation_map(0,0,50,50,0.1);

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
int main(int argc, char **argv);
