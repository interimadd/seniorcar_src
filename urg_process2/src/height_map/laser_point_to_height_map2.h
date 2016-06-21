#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
#include "height_map2.h"
#include <tf/transform_listener.h>
#include <math.h>
#include <algorithm>
/*
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
*/
ros::Publisher heightmap_pub;
ros::Publisher result_pub;
tf::TransformListener* listener;
Height_Map height_map;

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
int main(int argc, char **argv);
