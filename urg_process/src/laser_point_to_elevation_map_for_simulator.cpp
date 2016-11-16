#include "laser_point_to_elevation_map_for_simulator.h"

int i = 0;
float last_calc_x = -100;
float last_calc_y = -100;
 
void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

	tf::StampedTransform transform;
  tf::StampedTransform transform_vehicle_front;
  tf::StampedTransform transform_read_map_center;
  tf::StampedTransform transform_base_link;
	
	try{
      listener->lookupTransform("/odom","/map_center",ros::Time(0), transform);
      listener->lookupTransform("/odom","/hokuyo_link",ros::Time(0), transform_vehicle_front);
      listener->lookupTransform("/odom","/read_map_center",ros::Time(0), transform_read_map_center);
      listener->lookupTransform("/odom","/base_link",ros::Time(0), transform_base_link);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

  double roll,pitch,yaw;
  tf::Matrix3x3 m(transform_base_link.getRotation());
  m.getRPY(roll,pitch,yaw);
  //cout << "r:" << roll << " p:" << pitch << " y:" << yaw << endl;

  geometry_msgs::Pose now_pos;
  now_pos.position.x = transform_base_link.getOrigin().x();
  now_pos.position.y = transform_base_link.getOrigin().y();
  now_pos.position.z = transform_base_link.getOrigin().z();
  now_pos.orientation.x = transform_base_link.getRotation().x();
  now_pos.orientation.y = transform_base_link.getRotation().y();
  now_pos.orientation.z = transform_base_link.getRotation().z();
  now_pos.orientation.w = transform_base_link.getRotation().w();

	elevation_map.RecordSensorData(*msg);

	sensor_msgs::PointCloud send_point_cloud,result_point_cloud,road_point_cloud,grass_point_cloud,others_point_cloud;
	send_point_cloud.header = msg->header;
  result_point_cloud.header = msg->header;
  road_point_cloud.header = msg->header;
  grass_point_cloud.header = msg->header;
  others_point_cloud.header = msg->header;

  if(++i%2==0){
    elevation_map.calculateRisk(now_pos, now_state);
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
  if(i%10==0){
    elevation_map.printElevationMapData();
  }
  */

  geometry_msgs::PoseArray send_pose_array;
  send_pose_array.header = msg->header;
  elevation_map.returnCalculatedVehicleState(&send_pose_array);
  pose_pub.publish(send_pose_array);
  
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
    printf("%f,%f,%f\n",transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
  }

}


void SeniorcarStateCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel){
  now_state.vehicle_velocity = cmd_vel->linear.x ;
  if( cmd_vel->linear.x > 0.001){
    now_state.steer_angle = atan( cmd_vel->angular.z * 0.9 / cmd_vel->linear.x );
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
  pose_pub   = n.advertise<geometry_msgs::PoseArray>("calculated_pose",1000);

  road_pub  = n.advertise<sensor_msgs::PointCloud>("road_map", 1000);
  grass_pub = n.advertise<sensor_msgs::PointCloud>("grass_map", 1000);
  others_pub = n.advertise<sensor_msgs::PointCloud>("other_map", 1000);

  ros::Subscriber sub = n.subscribe("laser_point", 1000, PointCloudCallback);
  ros::Subscriber state_sub = n.subscribe("cmd_vel",1000,SeniorcarStateCallback);

  printf("start3\n");

  ros::spin();
  
  return 0;

}