#include "laser_point_to_elevation_map_ver2.h"

int step_count = 0;
float last_calc_x = -100;
float last_calc_y = -100;


void PointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
  tmp_header = msg->header;
  tmp_header.frame_id = "/odom";
  sensor_msgs::PointCloud record_point;
  elevation_map.RecordSensorData(*msg);
}


void publishResult(){

  tf::StampedTransform transform;
  try{
      listener->lookupTransform("/odom","/base_link",ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  tf::Matrix3x3 m(transform.getRotation());
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  //cout << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," <<  transform.getOrigin().z() << "," <<  roll << "," <<  pitch << "," <<  yaw << endl;
  elevation_map.predictAccident(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), roll, pitch, yaw);
  
  sensor_msgs::PointCloud send_point_cloud;
  send_point_cloud.header = tmp_header;
  elevation_map.HeightMapToPointCloud(&send_point_cloud);
  heightmap_pub.publish(send_point_cloud);

  visualization_msgs::Marker marker,marker2;
  marker.header.stamp = marker2.header.stamp = ros::Time::now();
  elevation_map.returnObjectMapPointsMarker(&marker2);
  elevation_map.returnCalculatedVehicleState(&marker);
  marker_pub.publish(marker);
  marker_pub.publish(marker2);

  PredictResult predict_result = elevation_map.returnPredictResult();
  ultimate_seniorcar::AccidentPredictResult result_for_pub;
  cout << "start" << endl;
  for(int i = 0; i < DEG_CALCULTE_NUM ; i++){
    cout << "steer_angle:" << predict_result.steer_angle[i] << "  max_distance:"<< predict_result.max_distance_to_go[i] << endl;
    result_for_pub.steer_angle.push_back(predict_result.steer_angle[i]);
    result_for_pub.max_distance.push_back(predict_result.max_distance_to_go[i]);
  }
  accident_predict_pub.publish(result_for_pub);
  
}


void moveMapCenter(){
  tf::StampedTransform transform;
  try{
    listener->lookupTransform("/odom","/map_center",ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }

  if( pow(elevation_map.center_x-transform.getOrigin().x(),2) + pow(elevation_map.center_y-transform.getOrigin().y(),2) > 1.0 ){
    elevation_map.MoveHeightMapCenter(transform.getOrigin().x(), transform.getOrigin().y());
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_point_to_elevation_map");
  ros::NodeHandle n;

  tf::TransformListener lr(ros::Duration(10));
  listener = &lr;

  ros::Subscriber sub = n.subscribe("laser_point", 1000, PointCloudCallback);

  heightmap_pub = n.advertise<sensor_msgs::PointCloud>("height_map", 1000);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  accident_predict_pub = n.advertise<ultimate_seniorcar::AccidentPredictResult>("accident_predict",10);

  ros::Rate loop_rate(20);
  
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

  return 0;

}