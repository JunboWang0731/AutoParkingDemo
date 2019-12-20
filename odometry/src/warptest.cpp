#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <math.h>
#include <vector>

using namespace std;
using namespace cv;
using namespace Eigen;

class SubAndPub
{
public:
  SubAndPub(Eigen::Matrix3d paramK, Eigen::Matrix3d paramKinv, Eigen::Matrix3d paramR)
  {
    K = paramK;
    Kinv = paramKinv;
    R = paramR.inverse();
    image_transport::ImageTransport itPub(n);
    image_transport::ImageTransport itSub(n);
    pub = itPub.advertise("image_warped", 1);
    sub = itSub.subscribe("/usb_cam/image_raw", 1, &SubAndPub::callback, this);
  }
  void callback( const sensor_msgs::ImageConstPtr& msg )
  {
    sensor_msgs::ImagePtr msgPub;
    cv::Mat Image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat Result = cv::Mat::zeros(Image.rows, Image.cols, CV_8UC3);
    Eigen::Matrix3d Homo = K * R * Kinv;
    cv::Matx33d H;
    eigen2cv(Homo, H);
    cv::warpPerspective(Image, Result, H, Result.size());
    msgPub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Result).toImageMsg();
    pub.publish(msgPub);
  }
private:
  ros::NodeHandle n;
  //ros::NodeHandle nSub;
  //image_transport::ImageTransport itPub;
  //image_transport::ImageTransport itSub;
  image_transport::Publisher pub;
  image_transport::Subscriber sub;
  Eigen::Matrix3d K;
  Eigen::Matrix3d Kinv;
  Eigen::Matrix3d R;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "warp");
  ros::NodeHandle nh;


  XmlRpc::XmlRpcValue paramList;
  std::vector<double> Intrinsic;
  std::vector<double> Rotation;
  Matrix3d K, Kinv, R;
  
  if(!ros::param::get("warp/Intrinsic", paramList))
  {
    ROS_ERROR("Failed to get parameter from server!");
    return -1;
  }
  else
  {
    for(size_t i = 0; i < paramList.size(); ++i)
    {
      XmlRpc::XmlRpcValue temp = paramList[i];
      if(temp.getType() == XmlRpc::XmlRpcValue::TypeDouble)
	Intrinsic.push_back(double(temp));
    }
  }
  
  
  cv::Matx33d matIntrinsic( 
  Intrinsic[0], Intrinsic[1], Intrinsic[2],
  Intrinsic[3], Intrinsic[4], Intrinsic[5],
  Intrinsic[6], Intrinsic[7], Intrinsic[8]);
  
  if(!ros::param::get("warp/Rotation", paramList))
  {
    ROS_ERROR("Failed to get parameter from server!");
    return -1;
  }
  else
  {
    for(size_t i = 0; i < paramList.size(); ++i)
    {
      XmlRpc::XmlRpcValue temp = paramList[i];
      if(temp.getType() == XmlRpc::XmlRpcValue::TypeDouble)
	Rotation.push_back(double(temp));
    }
  }

  cv::Matx33d matRotation( 
  Rotation[0], Rotation[1], Rotation[2],
  Rotation[3], Rotation[4], Rotation[5],
  Rotation[6], Rotation[7], Rotation[8]);

  cv2eigen(matIntrinsic, K);
  Kinv = K.inverse();
  cv2eigen(matRotation, R);
  
  SubAndPub SAP(K, Kinv, R);
  ros::spin();
  cout << "Intrinsic: " << endl << K << endl << "Rotation" << endl << R << endl;
  return 0;
}
