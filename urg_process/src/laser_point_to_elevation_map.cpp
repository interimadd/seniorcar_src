#include "laser_point_to_elevation_map.h"

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

	elevation_map.RecordSensorData(*msg);

	sensor_msgs::PointCloud send_point_cloud,result_point_cloud,road_point_cloud,grass_point_cloud,others_point_cloud;
	send_point_cloud.header = msg->header;
  result_point_cloud.header = msg->header;
  road_point_cloud.header = msg->header;
  grass_point_cloud.header = msg->header;
  others_point_cloud.header = msg->header;

  if(++i%2==0){
    elevation_map.VarianceMapToPointCloud(&result_point_cloud);
    result_pub.publish(result_point_cloud);
    elevation_map.HeightMapToPointCloud(&send_point_cloud);
    heightmap_pub.publish(send_point_cloud);
    elevation_map.TypeMapToPointCloud(&road_point_cloud, &grass_point_cloud, &others_point_cloud);
    road_pub.publish(road_point_cloud);
    grass_pub.publish(grass_point_cloud);
    others_pub.publish(others_point_cloud);
  }
  
  /*
  if(++i%3==0){
    height_map.UpdateRotateEnableMap();
    height_map.DetectRotateEnableAreaFromPoint(transform_vehicle_front.getOrigin().x(), transform_vehicle_front.getOrigin().y());
    height_map.RotateEnableAreaToPointCloud(&result_point_cloud);
    result_pub.publish(result_point_cloud);
  }
  */
  if( pow(elevation_map.center_x-transform.getOrigin().x(),2) + pow(elevation_map.center_y-transform.getOrigin().y(),2) > 1.0 ){
    elevation_map.MoveHeightMapCenter(transform.getOrigin().x(), transform.getOrigin().y());
    //printf("%f,%f,%f\n",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  }


}


int main(int argc, char **argv)
{

  printf("start\n");
 
  ros::init(argc, argv, "laser_point_to_elevation_map");
  ros::NodeHandle n;

  tf::TransformListener lr(ros::Duration(10));
  listener = &lr;

  printf("start2\n");

  heightmap_pub = n.advertise<sensor_msgs::PointCloud>("height_map", 1000);
  result_pub = n.advertise<sensor_msgs::PointCloud>("variance_map", 1000);

  road_pub  = n.advertise<sensor_msgs::PointCloud>("road_map", 1000);
  grass_pub = n.advertise<sensor_msgs::PointCloud>("grass_map", 1000);
  others_pub = n.advertise<sensor_msgs::PointCloud>("other_map", 1000);

  ros::Subscriber sub = n.subscribe("laser_point", 1000, PointCloudCallback);

  printf("start3\n");

  ros::spin();
  
  return 0;

}