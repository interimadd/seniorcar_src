#include "laser_point_to_voxel_map.h"

int step_count = 0;
float last_calc_x = -100;
float last_calc_y = -100;


void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

  tf::StampedTransform transform;
  try{
      listener->lookupTransform("/odom","/laser_tilt",ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  geometry_msgs::Point laser_tilt_corfinate;
  laser_tilt_corfinate.x = transform.getOrigin().x();
  laser_tilt_corfinate.y = transform.getOrigin().y();
  laser_tilt_corfinate.z = transform.getOrigin().z();

  tmp_header = msg->header;
  tmp_header.frame_id = "/odom";

  penetration_voxel_map.RecordSensorData(laser_tilt_corfinate,*msg);

}


void publishResult(){

  sensor_msgs::PointCloud send_point_cloud;
  send_point_cloud.header = tmp_header;
  penetration_voxel_map.VoxelMapToPointCloud(&send_point_cloud);
  heightmap_pub.publish(send_point_cloud);
  
}


void moveMapCenter(){
  tf::StampedTransform transform;
  try{
    listener->lookupTransform("/odom","/map_center",ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  if( pow(penetration_voxel_map.center_x-transform.getOrigin().x(),2) + pow(penetration_voxel_map.center_y-transform.getOrigin().y(),2) > 1.0 ){
    penetration_voxel_map.MoveVoxelMapCenter(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  }
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_point_to_voxel_map");
  ros::NodeHandle n;

  tf::TransformListener lr(ros::Duration(10));
  listener = &lr;

  ros::Subscriber sub = n.subscribe("laser_point", 1000, PointCloudCallback);

  heightmap_pub = n.advertise<sensor_msgs::PointCloud>("height_map", 1000);
  accident_predict_pub = n.advertise<ultimate_seniorcar::AccidentPredictResult>("accident_predict",10);

  ros::Rate loop_rate(50);
  
  while(ros::ok()){ 
    if( step_count % 10 == 0 ){ 
      publishResult();
      moveMapCenter();
    }
    step_count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  ros::spin();

  penetration_voxel_map.printVoxelMapData();

  return 0;

}