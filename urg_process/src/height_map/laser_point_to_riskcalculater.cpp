#include "laser_point_to_riskcalculater.h"

int i = 0;
bool start_flag = true;

void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){

  if(start_flag){
    start_time = ros::Time::now();
    cout << "aaaaaaaaaaaaaaaaaaaaaaaaaaa" << start_time.sec << endl ;
    start_flag = false;
  }

	tf::StampedTransform transform;
  tf::StampedTransform transform_vehicle;
	
	try{
      listener->lookupTransform("/odom","/map_center",ros::Time(0), transform);
      listener->lookupTransform("/odom","/base_link",ros::Time(0), transform_vehicle);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

  geometry_msgs::Pose now_pos;
  now_pos = now_odom.pose.pose;

	calculater.RecordSensorData(*msg);

  
	sensor_msgs::PointCloud send_point_cloud;
	send_point_cloud.header = msg->header;
  calculater.HeightMapToPointCloud(&send_point_cloud);
  heightmap_pub.publish(send_point_cloud);

  if(++i%2==0){
    ros::Time now = ros::Time::now();
    printf("time,%d.%09d,",now.sec,now.nsec);
    calculater.calculateRisk(now_pos,now_state);
  }

  geometry_msgs::PoseArray send_pose_array;
  send_pose_array.header = msg->header;
  calculater.returnCalculatedVehicleState(&send_pose_array);
  result_pub.publish(send_pose_array);
  
  /*
  if(++i%3==0){
    height_map.UpdateRotateEnableMap();
    height_map.DetectRotateEnableAreaFromPoint(transform_vehicle_front.getOrigin().x(), transform_vehicle_front.getOrigin().y());
    height_map.RotateEnableAreaToPointCloud(&result_point_cloud);
    result_pub.publish(result_point_cloud);
  }
  */
  if( pow(calculater.center_x-transform.getOrigin().x(),2) + pow(calculater.center_y-transform.getOrigin().y(),2) > 1.0 ){
    calculater.MoveHeightMapCenter(transform.getOrigin().x(), transform.getOrigin().y());
  }


}

void SeniorcarStateCallback(const ultimate_seniorcar::SeniorcarState& msg){
  now_state = msg;
};


void OdmetryCallback(const nav_msgs::Odometry& msg){
  now_odom = msg;
};


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "laser_point_to_riskcalculater");
  ros::NodeHandle n;

  tf::TransformListener lr(ros::Duration(10));
  listener = &lr;

  calculater.SetTireRadius(0.14, 0.2);
  heightmap_pub = n.advertise<sensor_msgs::PointCloud>("height_map", 1000);
  result_pub   = n.advertise<geometry_msgs::PoseArray>("calculated_pose",1000);
  ros::Subscriber sub = n.subscribe("laser_point", 1000, PointCloudCallback);
  ros::Subscriber state_sub = n.subscribe("seniorcar_state", 1000, SeniorcarStateCallback);
  ros::Subscriber odom_sub = n.subscribe("seniorcar_odometry", 1000, OdmetryCallback); 

  ros::spin();
  
  return 0;

}