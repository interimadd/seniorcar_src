#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>
#include <iostream>
using namespace std;

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
  cout << ros::Duration() << " " << ros::Time(0) << " " << scan_in->header.stamp << endl;
  
  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "/odom",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }
  
  cout << ros::Duration() << " " << ros::Time(0) << " " << scan_in->header.stamp << endl;

  sensor_msgs::PointCloud cloud;
  projector_.transformLaserScanToPointCloud("/odom",*scan_in,cloud,listener_);

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