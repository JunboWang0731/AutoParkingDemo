#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace cv;

class SubAndPub
{
public:
  SubAndPub()
  {
    image_transport::ImageTransport itSub(nh);
    image_transport::ImageTransport itPub0(nh);
    image_transport::ImageTransport itPub1(nh);
    diffPub = itPub0.advertise("/diff", 1);
    movingPub = itPub1.advertise("/moving", 1);
    imgSub = itSub.subscribe("/usb_cam_1/image_1", 1, &SubAndPub::callback, this);
    odoPub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    background = imread("/home/nvidia/apriltag_ws/src/movingdetect/back.png");
  }
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
    ROS_INFO("New Image!");
    count ++;
    if(count == 1)
    {
      //cvtColor(cv_bridge::toCvShare(msg, "bgr8")->image, background, cv::COLOR_BGR2GRAY);//load the first image
    }
    else
    {
      moving = cv_bridge::toCvShare(msg, "bgr8")->image;
      massCenter = CalculateMassCenter(moving, background, threshold);
      sensor_msgs::ImagePtr msgPub0 = cv_bridge::CvImage(std_msgs::Header(), "8UC1", diff).toImageMsg();
      sensor_msgs::ImagePtr msgPub1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", moving).toImageMsg();
      diffPub.publish(msgPub0);
      movingPub.publish(msgPub1);
    }
  }
  CvPoint CalculateMassCenter(Mat moving, Mat background, int threshold)
  {
	int row = background.rows;
	int col = background.cols;
	diff = moving.clone();
	cvtColor(diff, diff, CV_RGB2GRAY);
	for(int i = 0; i < row; i++)
	{
	  uchar *diffdata = diff.ptr<uchar>(i);
	  uchar *databack = background.ptr<uchar>(i);
	  uchar *datamoving = moving.ptr<uchar>(i);
	  for(int j = 0; j < col; j++)
	  {
	    int B = abs(datamoving[3*j+0] - databack[3*j+0]);
	    int G = abs(datamoving[3*j+1] - databack[3*j+1]);
	    int R = abs(datamoving[3*j+2] - databack[3*j+2]);
	    if((B>=threshold)||(G>=threshold)||(R>=threshold))
	    {
		if((datamoving[3*j+0]<=100)&&(datamoving[3*j+1]<=100)&&(datamoving[3*j+2]<=100))
		{
			diffdata[j] = 0;//delete the  black points from wheel
		}
		else
		{
	        	diffdata[j] = 255;//White point represent car
		}
	    }
	    else diffdata[j] = 0;
	  }
	}

	Mat element = getStructuringElement(MORPH_RECT, Size(4,4));
	morphologyEx(diff, diff, MORPH_CLOSE, element);//delete the noise(isolated white points)
	morphologyEx(diff, diff, MORPH_OPEN, element);//fill the white blank inside the car

	//Find rectangle
	vector< vector<cv::Point> > contours;  
    	vector<Vec4i> hierarchy;
	vector<cv::Point> tempcontour;
    	findContours(diff, contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE); //查找轮廓
    	for(int i=0; i<contours.size() - 1; i++)
    	{
		for(int j = 0; j<contours.size() - 1 - i; j++)
		{
			double ConArea1 = abs(contourArea(contours[j], true));
			double ConArea2 = abs(contourArea(contours[j+1], true));
			if(ConArea1 >= ConArea2)
			{
				tempcontour = contours[j];
				contours[j] = contours[j+1];
				contours[j+1] = tempcontour;	
			}
		}
	}//bubblesort to find the contour with maxmum area(also avoid the bad influence from noise)
	double ConArea = abs(contourArea(contours[contours.size() - 1], true));	
	cout << "【用轮廓面积计算函数计算出来最大轮廓的面积为：】" << ConArea << endl;
	RotatedRect rect = minAreaRect(contours[contours.size() - 1]);
	Point2f P[4];
	rect.points(P);
	for (int j = 0; j <= 3; j++)
	{
		line(moving, P[j], P[(j + 1) % 4], Scalar(0,0,255), 1);
		line(moving, P[j], P[(j + 1) % 4], Scalar(111), 2);
	}
    	

	/*int mc_x(0), count_x(0);
	int mc_y(0), count_y(0);
	for (int i = 0; i < row; i++)
	{
		uchar *data = diff.ptr<uchar>(i);
		for (int j = 0; j < col; j++)
		{
			if(data[j] == 0)
			{
				mc_x += j;
				mc_y += i;
				count_x += 1;
				count_y += 1;
			}
		}
	}
	mc_x = mc_x / count_x;
	mc_y = mc_y / count_y;
	CvPoint point = cvPoint(mc_x, mc_y);*/
	CvPoint point(1,1);
	return point;
  }
  /*void parameterGENERATE(Eigen::Matrix3d &RCam0Init, Eigen::Matrix3d &RCam0Covision, Eigen::Matrix3d &RCam1Covision, 
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
    
    Eigen::Matrix3d temp0, temp1, temp2, temp3;
    QCam0Covision.normalized();
    QCam1Covision.normalized();
    temp0 = QCam0Covision.toRotationMatrix();
    temp1 = QCam1Covision.toRotationMatrix();
    temp2 = temp0.inverse() * temp1;
    QCam0Init.normalized();
    temp0 = QCam0Init.toRotationMatrix();
    temp1 = temp0 * temp2;
    QCam1Init = (temp1);
    
    //TCam1Init = TCam0Init + ( - TCam0Covision + TCam1Covision );   
    temp0 = QCam0Covision.toRotationMatrix();
    temp1 = QCam0Init.toRotationMatrix();
    temp2 = temp1 * temp0.inverse();//The orintation of middle tag to initial position
    temp3 = QCam1Covision.toRotationMatrix();
    
    TCam1Init = temp2 * TCam1Covision + temp1 * temp0.inverse() * (- TCam0Covision) + TCam0Init;
  }*/
  
private:
  ros::NodeHandle nh;
  image_transport::Subscriber imgSub;
  //image_transport::Publisher imgPub;
  image_transport::Publisher diffPub;
  image_transport::Publisher movingPub;
  cv::Mat background;
  cv::Mat moving;
  cv::Mat diff;//The difference image between background and moving.
  CvPoint massCenter;
  ros::Publisher odoPub;
  nav_msgs::Odometry odo;
  int count = 0;
  int threshold = 50;
  /*Eigen::Matrix3d RCam0Init, RCam0Covision, RCam1Covision;
  Eigen::Vector3d TCam0Init, TCam0Covision, TCam1Covision, TCam1Init;  
  Eigen::Quaterniond QCam0Init, QCam0Covision, QCam1Covision, QCam1Init;*/
};



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "movingdetect");
	SubAndPub SAP;
	cout << "Initialization finished!" << endl;
	ros::spin();
	return 0;
}
