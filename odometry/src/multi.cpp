#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
using namespace std;
using namespace cv;
class SubAndPub
{
public:
  SubAndPub()
  {
    flag0 = 0;
    flag1 = 0;//flags are indicators of TAG, flag0 = 1 means TAG shows in cam0's vision
    count0 = 0;
    count1 = 0;
    calibrate = false;//true means the current mode is calibration( loacte dual cams in OXY frame of map ) 
    pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);//sub0&1 subscribe the odometry meassgae from cam0&1, /odom is the message after processing
    sub0 = nh.subscribe("/usb_cam_0/odometry_0", 1, &SubAndPub::callback0, this);
    sub1 = nh.subscribe("/usb_cam_1/odometry_1", 1, &SubAndPub::callback1, this); 
  }
  void parameterWRITE(const nav_msgs::Odometry odo, int type, int index)//Write the calibrating parameters into YAML file
  {
    cv::Mat mat = (cv::Mat_<double>(1,4) << odo.pose.pose.orientation.w, odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z);
    cv::Mat vec = (cv::Mat_<double>(1,3) << odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z);
    FileStorage fs("/home/junbo/dev/ROS_Tutorials/src/odometry/test.yaml", FileStorage::APPEND);
    if(type == 0)
    {
      fs << "QuatCam0Init" << mat;
      fs << "TranVecCam0Init" << vec;
      fs.release();
      cout << "WRITTEN!" << endl;
    }
    else if(index == 0)
    {
      fs << "QuatCam0Covision" << mat;
      fs << "TranVecCam0Covision" << vec;
      fs.release();
      cout << "WRITTEN!" << endl;
    }
    else
    {
      fs << "QuatCam1Covision" << mat;
      fs << "TranVecCam1Covision" << vec;
      fs.release();
      cout << "WRITTEN!" << endl;
    }
  }
  void parameterGENERATE(Eigen::Matrix3d &RCam0Init, Eigen::Matrix3d &RCam0Covision, Eigen::Matrix3d &RCam1Covision, 
			 Eigen::Vector3d &TCam0Init, Eigen::Vector3d &TCam0Covision, Eigen::Vector3d &TCam1Covision, 
			  Eigen::Quaterniond &QCam0Init, Eigen::Quaterniond &QCam0Covision, Eigen::Quaterniond &QCam1Covision,
			  Eigen::Quaterniond &QCam1Init, Eigen::Vector3d &TCam1Init
  )//Read the calibration information from YAML file(Note that in OpenCV 3.3 the generated file DO NOT contain '%YAML:1.0' so it should added manually)
  {
    FileStorage fs("/home/junbo/dev/ROS_Tutorials/src/odometry/test.yaml", FileStorage::READ);
    if(!fs.isOpened())
    {
      std::cout << "No file!" << std::endl;
      //return -1;
    }
    cv::Mat Q0I, Q0C, Q1C;
    cv::Mat T0I, T0C, T1C;
    fs["QuatCam0Init"] >> Q0I;
    fs["QuatCam0Covision"] >> Q0C;
    fs["QuatCam1Covision"] >> Q1C;
    fs["TranVecCam0Init"] >> T0I;
    fs["TranVecCam0Covision"] >> T0C;
    fs["TranVecCam1Covision"] >> T1C;
    fs.release();
    
    TCam0Init << T0I.at<double>(0,0), T0I.at<double>(0,1), T0I.at<double>(0,2);
    TCam0Covision << T0C.at<double>(0,0), T0C.at<double>(0,1), T0C.at<double>(0,2);
    TCam1Covision << T1C.at<double>(0,0), T1C.at<double>(0,1), T1C.at<double>(0,2);
    
    QCam0Init.w() = Q0I.at<double>(0,0);
    QCam0Init.x() = Q0I.at<double>(0,1);
    QCam0Init.y() = Q0I.at<double>(0,2);
    QCam0Init.z() = Q0I.at<double>(0,3);
    
    QCam0Covision.w() = Q0C.at<double>(0,0);
    QCam0Covision.x() = Q0C.at<double>(0,1);
    QCam0Covision.y() = Q0C.at<double>(0,2);
    QCam0Covision.z() = Q0C.at<double>(0,3);
    
    QCam1Covision.w() = Q1C.at<double>(0,0);
    QCam1Covision.x() = Q1C.at<double>(0,1);
    QCam1Covision.y() = Q1C.at<double>(0,2);
    QCam1Covision.z() = Q1C.at<double>(0,3);
    
    Eigen::Matrix3d temp0, temp1, temp2;
    QCam0Covision.normalized();
    QCam1Covision.normalized();
    temp0 = QCam0Covision.toRotationMatrix();
    temp1 = QCam1Covision.toRotationMatrix();
    temp2 = temp0.inverse() * temp1;
    QCam0Init.normalized();
    temp0 = QCam0Init.toRotationMatrix();
    temp1 = temp0 * temp2;
    QCam1Init = (temp1);
    
    TCam1Init = TCam0Init + ( - TCam0Covision + TCam1Covision );   
  }
  void callback0(const nav_msgs::Odometry &msg0)
  {
    flag0 = 1;
    count0 ++;
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
      //pub.publish(odo0);
      ROS_INFO("FROM CAM 0!");
      if(calibrate)
      {
	if(count0 == 100)
	{
	  parameterWRITE(odo0, 0, 0);
	}
      }
      else
      {
	if(count0 == 1)
	{
	  parameterGENERATE(RCam0Init, RCam0Covision, RCam1Covision, TCam0Init, TCam0Covision, TCam1Covision, QCam0Init, QCam0Covision, QCam1Covision, QCam1Init, TCam1Init);
	}
	//Ready to output
	Eigen::Quaterniond Qtemp(msg0.pose.pose.orientation.w, 
	  msg0.pose.pose.orientation.x,
	  msg0.pose.pose.orientation.y,
	  msg0.pose.pose.orientation.z
	);
	Qtemp.normalized();
	QCam0Init.normalized();
	Eigen::Matrix3d MTag = Qtemp.toRotationMatrix();
	Eigen::Matrix3d MCam0Init = QCam0Init.toRotationMatrix();
	Eigen::Matrix3d MTagInit = MCam0Init * MTag.inverse();
	Eigen::Quaterniond QTagInit( MTagInit );
	odo0.pose.pose.orientation.w = QTagInit.w();
	odo0.pose.pose.orientation.x = QTagInit.x();
	odo0.pose.pose.orientation.y = QTagInit.y();
	odo0.pose.pose.orientation.z = QTagInit.z();
	
	Eigen::Vector3d first(1, 0, 0);
	Eigen::Vector3d second(0, 1, 0);
	odo0.pose.pose.position.x = (- odo0.pose.pose.position.x) + (first.transpose() * TCam0Init);
	odo0.pose.pose.position.y = (- odo0.pose.pose.position.y) + (second.transpose() * TCam0Init);
	pub.publish(odo0);
	//cout << (first.transpose() * TCam0Init) << endl;
      }
    }
  }
  void callback1(const nav_msgs::Odometry &msg1)
  {
    flag1 = 1;
    count1 ++;
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
      //pub.publish(odo1);
      ROS_INFO("FROM CAM 1!");
      if(calibrate)
      {
	if(count1 == 100)
	{
	  parameterWRITE(odo0, 1, 0);
	  parameterWRITE(odo1, 1, 1);
	}
      }
      else
      {
	Eigen::Quaterniond Qtemp(msg1.pose.pose.orientation.w, 
	  msg1.pose.pose.orientation.x,
	  msg1.pose.pose.orientation.y,
	  msg1.pose.pose.orientation.z
	);
	Qtemp.normalized();
	QCam1Init.normalized();
	Eigen::Matrix3d MTag = Qtemp.toRotationMatrix();
	Eigen::Matrix3d MCam1Init = QCam1Init.toRotationMatrix();
	Eigen::Matrix3d MTagInit = MCam1Init * MTag.inverse();
	Eigen::Quaterniond QTagInit( MTagInit );
	odo1.pose.pose.orientation.w = QTagInit.w();
	odo1.pose.pose.orientation.x = QTagInit.x();
	odo1.pose.pose.orientation.y = QTagInit.y();
	odo1.pose.pose.orientation.z = QTagInit.z();
	
	Eigen::Vector3d first(1, 0, 0);
	Eigen::Vector3d second(0, 1, 0);
	odo1.pose.pose.position.x = (- odo1.pose.pose.position.x) + (first.transpose() * TCam1Init);
	odo1.pose.pose.position.y = (- odo1.pose.pose.position.y) + (second.transpose() * TCam1Init);
	pub.publish(odo1);
      }
    }
  }
  bool calibrate;
private:
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub0;
  ros::Subscriber sub1;
  nav_msgs::Odometry odo0;
  nav_msgs::Odometry odo1;
  int count0, count1;
  bool flag0, flag1;
  Eigen::Matrix3d RCam0Init, RCam0Covision, RCam1Covision;
  Eigen::Vector3d TCam0Init, TCam0Covision, TCam1Covision, TCam1Init;  
  Eigen::Quaterniond QCam0Init, QCam0Covision, QCam1Covision, QCam1Init;
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "odocollector");
  SubAndPub sap;
  if(argc == 2)
  {
    string str(argv[1]);
    str += '\\0';
    if( str.compare("calibrate") )
    {
      sap.calibrate = true;
    }
  }
  cout << "start!" << endl;
  ros::MultiThreadedSpinner s(2);
  ros::spin(s);//Starting multi thread so the callback0 and callback1 will work at same time
  
  return 0; 
}
