#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
using namespace std;
class SubAndPub
{
public:
  SubAndPub()
  {
    flag0 = 0;
    flag1 = 0;
    pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    sub0 = nh.subscribe("/usb_cam_0/odometry_0", 1, &SubAndPub::callback0, this);
    sub1 = nh.subscribe("/usb_cam_1/odometry_1", 1, &SubAndPub::callback1, this); 
  }
  void callback0(const nav_msgs::Odometry &msg0)
  {
    flag0 = 1;
    odo0.header = msg0.header;
    odo0.pose.pose.position.x = msg0.pose.pose.position.x;// - init_x;
    odo0.pose.pose.position.y = msg0.pose.pose.position.y;// - init_y;
    odo0.pose.pose.position.z = msg0.pose.pose.position.z;
    odo0.pose.pose.orientation.w = msg0.pose.pose.orientation.w;
    odo0.pose.pose.orientation.x = msg0.pose.pose.orientation.x;
    odo0.pose.pose.orientation.y = msg0.pose.pose.orientation.y;
    odo0.pose.pose.orientation.z = msg0.pose.pose.orientation.z;
    odo0.twist.twist.linear.x = 0.0;
    odo0.twist.twist.linear.y = 0.0;
    odo0.twist.twist.linear.z = 0.0;
    odo0.twist.twist.angular.x = 0.0;
    odo0.twist.twist.angular.y = 0.0;
    odo0.twist.twist.angular.z = 0.0;
    cout << "The flags: " << flag0 << endl << flag1 << endl;
    if(flag0 == 1 && flag1 == 0)
    {
      pub.publish(odo0);
      ROS_INFO("FROM CAM 0!");
    }
  }
  void callback1(const nav_msgs::Odometry &msg1)
  {
    flag1 = 1;
    flag1_old = flag1;
    odo1.header = msg1.header;
    odo1.pose.pose.position.x = msg1.pose.pose.position.x;// - init_x;
    odo1.pose.pose.position.y = msg1.pose.pose.position.y;// - init_y;
    odo1.pose.pose.position.z = msg1.pose.pose.position.z;
    odo1.pose.pose.orientation.w = msg1.pose.pose.orientation.w;
    odo1.pose.pose.orientation.x = msg1.pose.pose.orientation.x;
    odo1.pose.pose.orientation.y = msg1.pose.pose.orientation.y;
    odo1.pose.pose.orientation.z = msg1.pose.pose.orientation.z;
    odo1.twist.twist.linear.x = 0.0;
    odo1.twist.twist.linear.y = 0.0;
    odo1.twist.twist.linear.z = 0.0;
    odo1.twist.twist.angular.x = 0.0;
    odo1.twist.twist.angular.y = 0.0;
    odo1.twist.twist.angular.z = 0.0;
    cout << "The flags: " << flag0 << endl << flag1 << endl;
    if(flag1 == 1)
    {
      pub.publish(odo1);
      ROS_INFO("FROM CAM 1!");
    }
  }
  /*void Filter()
  {
    if(flag0 || flag1 == 0)
    {
      ROS_INFO("NO TAGS RIGHT NOW!");
    }
    else if(flag0 && flag1 == 1)
    {
      ROS_INFO("TAGS SHOWS IN BOTH CAM!");
    }
    else
    {
      if(flag0 == 1)
      {
	ROS_INFO("TAGS SHOWS IN CAM0!");
      }
      else
      {
	ROS_INFO("TAGS SHOWS IN CAM1!");
      }
    }
  }*/
private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub0;
  ros::Subscriber sub1;
  nav_msgs::Odometry odo0;
  nav_msgs::Odometry odo1;
  //nav_msgs::Odometry odo;
  bool flag0, flag1;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odocollector");
  SubAndPub sap;
  cout << "start!" << endl;
  ros::MultiThreadedSpinner s(2);
  ros::spin(s);
  
  return 0; 
}
