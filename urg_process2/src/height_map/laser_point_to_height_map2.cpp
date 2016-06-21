#include "laser_point_to_height_map2.h"

int i = 0;

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
	tf::StampedTransform transform;
	tf::StampedTransform transform_vehicle_front;
	
	try{
		listener->lookupTransform("/odom","/map_center",ros::Time(0), transform);
		listener->lookupTransform("/odom","/laser_front",ros::Time(0), transform_vehicle_front);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}

	height_map.RecordSensorData(*msg);

	sensor_msgs::PointCloud send_point_cloud,result_point_cloud;
	send_point_cloud.header = msg->header;
  	result_point_cloud.header = msg->header;
	height_map.HeightMapToPointCloud(&send_point_cloud);
  	heightmap_pub.publish(send_point_cloud);
  	
  	/*
  	if(++i%6==0){
  		height_map.UpdateRotateEnableMap();
  		height_map.DetectRotateEnableAreaFromPoint(transform_vehicle_front.getOrigin().x(), transform_vehicle_front.getOrigin().y());
  		//height_map.MoveHeightMapCenter(transform.getOrigin().x(), transform.getOrigin().y());
  	}
  	*/
  	
  	if( sqrt(pow(height_map.center_x-transform.getOrigin().x(),2) + pow(height_map.center_y-transform.getOrigin().y(),2)) > 0.2 ){
  		height_map.MoveHeightMapCenter(transform.getOrigin().x(), transform.getOrigin().y());
  		height_map.UpdateRotateEnableMap();
  		//printf("%f,%f,%f\n",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  	}
  	
  	height_map.DetectRotateEnableAreaFromPoint(transform_vehicle_front.getOrigin().x(), transform_vehicle_front.getOrigin().y());
  	height_map.RotateEnableAreaToPointCloud(&result_point_cloud);
  	result_pub.publish(result_point_cloud);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_point_to_height_map2");
  ros::NodeHandle n;

  tf::TransformListener lr(ros::Duration(10));
  listener = &lr;
  
  heightmap_pub = n.advertise<sensor_msgs::PointCloud>("height_map", 1000);
  result_pub = n.advertise<sensor_msgs::PointCloud>("rotate_enable", 1000);
  ros::Subscriber sub = n.subscribe("laser_point", 1000, PointCloudCallback);

  ros::spin();
  
  return 0;

}
