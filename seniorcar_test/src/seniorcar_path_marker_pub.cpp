#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <ultimate_seniorcar/SeniorcarState.h>
#include <iostream>

const float SENIORCAR_WHEEL_BASE_LENGTH = 0.9;
const float calculate_distance_step = 0.1;
const float CALC_NUM = 30;

ultimate_seniorcar::SeniorcarState now_command,now_state;

void SeniorcarCommandCallback(const ultimate_seniorcar::SeniorcarState& msg){
  now_command = msg;
}

void SeniorcarStateCallback(const ultimate_seniorcar::SeniorcarState& msg){
  now_state = msg;
}

// 経路のマーカーを操舵角度から生成
void generatePathMarker(visualization_msgs::Marker *triangles,float angle,std_msgs::ColorRGBA rgba){

  float pos_x = 0.0;
  float pos_y = 0.0;
  float pos_z = 0.3;
  float yaw   = 0.0;

  triangles->header.frame_id = "/base_link";
  triangles->header.stamp = ros::Time::now();
  triangles->ns = "seniorcar_path_marker";
  triangles->action = visualization_msgs::Marker::ADD;
  triangles->pose.orientation.w = 1.0;
  triangles->id = 0;
  triangles->type = visualization_msgs::Marker::TRIANGLE_LIST;
  triangles->scale.x = triangles->scale.y = triangles->scale.z = 1.0f;

  geometry_msgs::Point p[3],tmp_p[3];

  float TRIANGLE_LENGTH = 0.1;
  tmp_p[0].x =  TRIANGLE_LENGTH; tmp_p[0].y =  0.0f;
  tmp_p[1].x = -TRIANGLE_LENGTH; tmp_p[1].y =  TRIANGLE_LENGTH/2.0f; 
  tmp_p[2].x = -TRIANGLE_LENGTH; tmp_p[2].y = -TRIANGLE_LENGTH/2.0f;

  for(int i = 1 ; i < CALC_NUM ; i++){
    yaw = calculate_distance_step * angle / SENIORCAR_WHEEL_BASE_LENGTH * float(i);
    pos_x += calculate_distance_step * cos(yaw);
    pos_y += calculate_distance_step * sin(yaw);
    for(int p_i=0;p_i<3;p_i++){
      p[p_i].x = pos_x + cos(yaw) * tmp_p[p_i].x - sin(yaw) * tmp_p[p_i].y;
      p[p_i].y = pos_y + sin(yaw) * tmp_p[p_i].x + cos(yaw) * tmp_p[p_i].y;
      p[p_i].z = pos_z ;
      triangles->points.push_back(p[p_i]);
      triangles->colors.push_back(rgba);
    }
  }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "seniorcar_path_marker_pub");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("seniorcar_path_marker", 1);
  ros::Subscriber command_sub = n.subscribe("seniorcar_command", 1000, SeniorcarCommandCallback);
  ros::Subscriber state_sub   = n.subscribe("seniorcar_state", 1000, SeniorcarStateCallback);

  std_msgs::ColorRGBA blue;
  blue.r = 0.0f; blue.g = 0.0f; blue.b = 1.0f; blue.a = 1.0f;
  std_msgs::ColorRGBA yellow;
  yellow.r = 1.0f; yellow.g = 1.0f; yellow.b = 0.0f; yellow.a = 1.0f;
  
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    generatePathMarker(&marker,now_state.steer_angle*3.14/180.0,yellow);
    generatePathMarker(&marker,now_command.steer_angle*3.14/180.0,blue);
    // Publish the marker
    marker_pub.publish(marker);
    ros::spinOnce();
    r.sleep();
  }
}