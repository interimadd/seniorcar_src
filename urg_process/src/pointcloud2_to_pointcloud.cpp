#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_listener.h>


class PointCloud2ToPointCloud{
  private:
    ros::Publisher pointcloud_pub;
    ros::NodeHandle n;
    ros::Subscriber sub;
    tf::TransformListener listener;
  public:
    PointCloud2ToPointCloud(int argc,char** argv);
    void start();
    void Callback(const sensor_msgs::PointCloud2ConstPtr& input);
};

void PointCloud2ToPointCloud::Callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
  sensor_msgs::PointCloud output , output_tf;
  sensor_msgs::convertPointCloud2ToPointCloud(*input,output);

  if(!listener.waitForTransform( output.header.frame_id, "/odom", 
    input->header.stamp + ros::Duration().fromSec(0.1), ros::Duration(1.0))){
    return;
  }  

  try{
      listener.transformPointCloud("/odom",output, output_tf);
      pointcloud_pub.publish(output_tf);
  }
  catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
  }

}

PointCloud2ToPointCloud::PointCloud2ToPointCloud(int argc,char** argv){
  pointcloud_pub = n.advertise<sensor_msgs::PointCloud>("laser_point", 1000); 
  sub = n.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 10, &PointCloud2ToPointCloud::Callback , this);
}

void PointCloud2ToPointCloud::start(){
  ros::spin();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pointclou2_to_pointcloud");  
  PointCloud2ToPointCloud manager(argc,argv);
  manager.start();
  return 0;
}