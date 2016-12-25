#include "ros/ros.h"
//#include "geometry_msgs/Point.h"
#include "sensor_msgs/PointCloud.h"
//#include "elevation_map_ver2/elevation_map_ver2.h"
//#include "elevation_map_ver2/accident_predictor.h"
#include "penetration_voxel_map/accident_predictor_for_penetration_voxel_map.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <math.h>
//#include <algorithm>
//#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_listener.h>
#include "ultimate_seniorcar/AccidentPredictResult.h"
#include <tf/transform_datatypes.h>
#include <math.h>
#include "geometry_msgs/PoseArray.h"

std_msgs::Header tmp_header;
ros::Publisher heightmap_pub;
ros::Publisher marker_pub;
ros::Publisher accident_predict_pub;

tf::TransformListener* listener;
ros::Time start_time;
ultimate_seniorcar::SeniorcarState now_state;

AccidentPredictor penetration_voxel_map(0,0,0);

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg);
void publishResult();
void predictTest();
int main(int argc, char **argv);
