#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <iostream>
using namespace std;

double MIN_DISTANCE = 0.5;  // Scanデータで近すぎるものを除去するときの距離の閾値
double NOT_DETECT   = 1000; // 短すぎるデータを置換して仮想的に除去する。置き換える距離

class ScanToPoint{
  private:
    laser_geometry::LaserProjection projector_;
    ros::Publisher pubTrackMarkers;
    ros::NodeHandle n;
    ros::Subscriber sub;
    tf::TransformListener listener_;
  public:
    ScanToPoint(int argc,char** argv);
    void start();
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
};


void ScanToPoint::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/odom",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }
  
  sensor_msgs::LaserScan scan_filtered = *scan_in;
  int data_num = ( scan_filtered.angle_max - scan_filtered.angle_min ) / scan_filtered.angle_increment;
  for(int i=0;i < 100; i++){
    if( scan_filtered.ranges[i] < MIN_DISTANCE ){
      scan_filtered.ranges[i] = NOT_DETECT;
    }
  }

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/odom",scan_filtered,cloud,listener_);

  pubTrackMarkers.publish(cloud);

  // Do something with cloud.
}

ScanToPoint::ScanToPoint(int argc,char** argv){
  pubTrackMarkers = n.advertise<sensor_msgs::PointCloud>("/laser_point",1);
}

void ScanToPoint::start(){
  string s;
  if( !n.getParam("tilt_laser_topic",s) ){
    s = "scan";
  }
  cout << s << endl;
  ros::Duration(1.0).sleep();
  sub = n.subscribe( s , 1000,&ScanToPoint::scanCallback,this);
  ros::spin();
}

int main( int argc, char** argv ){
  ros::init(argc, argv, "lase_scan_to_point_cloud");
  ScanToPoint manager(argc,argv);
  manager.start();
  return 0;
}