#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <boost/bind.hpp>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Dense>
#include <stdio.h>
#include <math.h>
#include <vector>

using namespace std;
using namespace message_filters;
//using namespace Eigen;

/*class OdoProcessor
{
public:
  OdoProcessor()
  {
    //odometrySub0 = nh.subscribe("usb_cam_0/odometry_0", 1, &OdoProcessor::callback, this);
    //odometrySub0 = nh.subscribe("usb_cam_1/odometry_1", 1, &OdoProcessor::callback, this);
    //odometrySub0(nh, "usb_cam_0/odometry_0", 1);             
    //odometrySub1(nh, "usb_cam_1/odometry_1", 1);   
    message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(odometrySub0, odometrySub1, 2);       
    sync.registerCallback(boost::bind(&OdoProcessor::callback, _1, _2));
    odometryPub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
  }
  void callback( nav_msgs::Odometry &odometry0,  nav_msgs::Odometry &odometry1)
  {
    nav_msgs::Odometry odoPubContainer;
    odoPubContainer.header = odometry0.header;
    odoPubContainer.pose.pose.position.x = odometry0.pose.pose.position.x;// - init_x;
    odoPubContainer.pose.pose.position.y = odometry0.pose.pose.position.y;// - init_y;
    odoPubContainer.pose.pose.position.z = odometry0.pose.pose.position.z;
    odoPubContainer.pose.pose.orientation.w = odometry0.pose.pose.orientation.w;
    odoPubContainer.pose.pose.orientation.x = odometry0.pose.pose.orientation.w;
    odoPubContainer.pose.pose.orientation.y = odometry0.pose.pose.orientation.w;
    odoPubContainer.pose.pose.orientation.z = odometry0.pose.pose.orientation.w;
    odoPubContainer.twist.twist.linear.x = 0.0;
    odoPubContainer.twist.twist.linear.y = 0.0;
    odoPubContainer.twist.twist.linear.z = 0.0;
    odoPubContainer.twist.twist.angular.x = 0.0;
    odoPubContainer.twist.twist.angular.y = 0.0;
    odoPubContainer.twist.twist.angular.z = 0.0;
    odometryPub.publish(odoPubContainer);
  }
private:
  ros::NodeHandle nh;
  //ros::Subscriber odometrySub0;
  //ros::Subscriber odometrySub1;
  message_filters::Subscriber<nav_msgs::Odometry> odometrySub0(nh, "usb_cam_0/odometry_0", 1);             
  message_filters::Subscriber<nav_msgs::Odometry> odometrySub1(nh, "usb_cam_1/odometry_1", 1);   
  
  ros::Publisher odometryPub;
};*/
void callback( const nav_msgs::Odometry odometry0,  const nav_msgs::Odometry odometry1)
  {
    nav_msgs::Odometry odoPubContainer;
    odoPubContainer.header = odometry0.header;
    odoPubContainer.pose.pose.position.x = odometry0.pose.pose.position.x;// - init_x;
    odoPubContainer.pose.pose.position.y = odometry0.pose.pose.position.y;// - init_y;
    odoPubContainer.pose.pose.position.z = odometry0.pose.pose.position.z;
    odoPubContainer.pose.pose.orientation.w = odometry0.pose.pose.orientation.w;
    odoPubContainer.pose.pose.orientation.x = odometry0.pose.pose.orientation.w;
    odoPubContainer.pose.pose.orientation.y = odometry0.pose.pose.orientation.w;
    odoPubContainer.pose.pose.orientation.z = odometry0.pose.pose.orientation.w;
    odoPubContainer.twist.twist.linear.x = 0.0;
    odoPubContainer.twist.twist.linear.y = 0.0;
    odoPubContainer.twist.twist.linear.z = 0.0;
    odoPubContainer.twist.twist.angular.x = 0.0;
    odoPubContainer.twist.twist.angular.y = 0.0;
    odoPubContainer.twist.twist.angular.z = 0.0;
    //odometryPub.publish(odoPubContainer);
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle nh;
  //ros::Subscriber odometrySub0;
  //ros::Subscriber odometrySub1;
  //ros::Publisher odometryPub;
  message_filters::Subscriber<nav_msgs::Odometry> odometrySub0(nh, "usb_cam_0/odometry_0", 1);             
  message_filters::Subscriber<nav_msgs::Odometry> odometrySub1(nh, "usb_cam_1/odometry_1", 1);   
  message_filters::TimeSynchronizer<nav_msgs::Odometry, nav_msgs::Odometry> sync(odometrySub0, odometrySub1, 2);       
  sync.registerCallback(boost::bind(&callback, _1, _2));
  //odometryPub = nh.advertise<nav_msgs::Odometry>("odometry", 1);
  
  bool calibrate = true;
  //OdoProcessor odometryFilter;
  if(calibrate)
  {
    ros::spin();
  }
  else
  {
    
  }
  
  return 0;
}
