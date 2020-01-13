#include "ros/ros.h"
#include "std_msgs/String.h"


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>  
#include <stdio.h>  

#include "opencv2/core.hpp"  
#include "opencv2/core/utility.hpp"  
#include "opencv2/core/ocl.hpp"  
#include "opencv2/imgcodecs.hpp"  
#include "opencv2/highgui.hpp"  
#include "opencv2/features2d.hpp"  
#include "opencv2/calib3d.hpp"  
#include "opencv2/imgproc.hpp"  
#include "opencv2/flann.hpp"  
#include "opencv2/xfeatures2d.hpp"  
#include "opencv2/ml.hpp"

#include <sstream>

#include <vector>
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace cv::ml;

typedef struct
{
	Point2f left_top;
	Point2f left_bottom;
	Point2f right_top;
	Point2f right_bottom;
}four_corners_t;
 
four_corners_t corners_ground, corners_car;

void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst, four_corners_t& corners);
 
//#define DEBUG

std::vector<cv::Rect> findParkingSpace(const cv::Mat& img){
    cv::Mat img_gray;
    cv::cvtColor(img,img_gray,cv::COLOR_BGR2GRAY);
    cv::equalizeHist(img_gray,img_gray);
    cv::GaussianBlur(img_gray,img_gray,{15,15},0.0);
//    cv::imshow("img_gray",img_gray);
//    cv::waitKey(0);
#ifdef DEBUG
    auto img_copy = img.clone();
#endif
    cv::Mat img_canny;
    cv::Canny(img_gray,img_canny,60,200);
    cv::rectangle(img_canny,{0,0,240,img.rows},{0,0,0},-1);
    cv::rectangle(img_canny,{985,0,img.cols-985,img.rows},{0,0,0},-1);
#ifdef DEBUG
    cv::imshow("canny",img_canny);
#endif
    std::vector<std::vector<cv::Point>> counters;
    cv::findContours(img_canny,counters,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_TC89_L1);
//    cv::imshow("img_canny",img_canny);
//    cv::waitKey(0);
    //找到所有停车位的roi
    cv::Rect rect_roi;
    std::vector<cv::Rect> rects;
    //找到可能的roi
    for(const auto& counter:counters){
        if(counter.size()>100){
            auto rect = cv::boundingRect(counter);
//            cv::rectangle(img_copy,rect,{0,255,255},2);
            if(rect.width>img.cols/3 and rect.height>img.rows/3){
                rects.push_back(rect);
            }
        }
    }
    //取最大的
    rect_roi = *std::max_element(rects.begin(),rects.end(),
                                 [](const cv::Rect& rect1,const cv::Rect& rect2){
        return rect1.area()<rect2.area();});
//    cv::imshow("img_copy",img_copy);
//    cv::waitKey(0);
    rect_roi.x-=2;
    rect_roi.width+=4;
    rect_roi.y-=2;
    rect_roi.height+=4;
    auto img_canny_roi = img_canny(rect_roi);
    std::vector<cv::Vec4f> lines;
    cv::HoughLinesP(img_canny_roi,lines,0.1,CV_PI/180,10,50,10);
#ifdef DEBUG
    //画直线
    auto roi_point = rect_roi.tl();//roi左上角的点
    for(const auto& line:lines){
        cv::Point pt1(line[0],line[1]);
        cv::Point pt2(line[2],line[3]);
        cv::line(img_copy,pt1+roi_point,pt2+roi_point,{65,252,0},2); // 线条宽度设置为2
    }
    cv::imshow("copy",img_copy);
#endif
    //找到所有的水平线和竖直线并排序
    std::vector<cv::Vec4f> vertical_lines,horizontal_lines;
    for(const auto& line:lines){
        if(line[0]==line[2]){
            vertical_lines.push_back(line);
        } else {
            auto k = std::abs((line[3]-line[1])/(line[2]-line[0]));
            if(k<0.2f){
                horizontal_lines.push_back(line);
            } else if (k>0.8f) {
                vertical_lines.push_back(line);
            }
        }
    }
    std::sort(vertical_lines.begin(),vertical_lines.end(),
              [](const cv::Vec4f& x,const cv::Vec4f& y){ return x[0]<y[0];}
    );
    std::sort(horizontal_lines.begin(),horizontal_lines.end(),
              [](const cv::Vec4f& x,const cv::Vec4f& y){ return x[1]<y[1];}
    );
    //合并相近的线
    std::vector<cv::Vec4f> vertical_merged_lines,horizontal_merged_lines;
    //垂直
    for(const auto& line:vertical_lines){
        if(vertical_merged_lines.empty()){
            vertical_merged_lines.push_back(line);
        } else {
            //两条线距离过近
            if(line[0]-vertical_merged_lines.back()[0]<20){
                for(auto i=0;i<4;++i){
                    vertical_merged_lines.back()[i]=(line[i]+vertical_merged_lines.back()[i])/2;
                }
            } else {
                vertical_merged_lines.push_back(line);
            }
        }
    }
    //水平
    for(const auto& line:horizontal_lines){
        if(horizontal_merged_lines.empty()){
            horizontal_merged_lines.push_back(line);
        } else {
            //两条线距离过近
            if(line[1]-horizontal_merged_lines.back()[1]<20){
                for(auto i=0;i<4;++i){
                    horizontal_merged_lines.back()[i]=(line[i]+horizontal_merged_lines.back()[i])/2;
                }
            } else {
                horizontal_merged_lines.push_back(line);
            }
        }
    }
    //整理数据
    std::vector<cv::Rect> ret_rects;
    if(horizontal_merged_lines.size()<2){
        return ret_rects;
    } else if (vertical_merged_lines.size()<2) {
        return ret_rects;
    }
    //auto y_center=rect_roi.y+rect_roi.height/2;
    auto special_parking_x=(horizontal_merged_lines[1][0]+horizontal_merged_lines[1][2])/2;
    for (auto l = vertical_merged_lines.cbegin();l!=vertical_merged_lines.cend()-1;++l) {
        cv::Rect p;
        if(horizontal_merged_lines.size()>2 and  //检测出多条水平线
                special_parking_x>(*l)[0] and special_parking_x<(*(l+1))[0]){ //特殊水平线的中心在两条竖直线中间被认为是横着的停车位
            p.x = static_cast<int>(((*l)[0]))+rect_roi.x;
            p.width = static_cast<int>(((*(l+1))[0])-(*l)[0]);
            p.y = static_cast<int>(horizontal_merged_lines[1][1]) + rect_roi.y;
            p.height = static_cast<int>(rect_roi.height - horizontal_merged_lines[1][1]);
        } else {
            p.x=static_cast<int>(((*l)[0]))+rect_roi.x;
            p.width = static_cast<int>(((*(l+1))[0])-(*l)[0]);
            p.y = rect_roi.y;
            p.height= static_cast<int>(rect_roi.height);
        }
        ret_rects.push_back(p);
    }
    return ret_rects;
}

std::vector<cv::Point> findParkingSpacePoints(const cv::Mat& img){
    auto rects = findParkingSpace(img);
    std::vector<cv::Point> points;
    for(const auto& rect:rects){
        points.emplace_back(rect.x+rect.width/2,rect.y+rect.height/2);
    }
    return points;
}

 
void CalcCorners(const Mat& H, const Mat& src, four_corners_t& corners)
{
	double v2[] = { 0, 0, 1 };//左上角
	double v1[3];//变换后的坐标值
	Mat V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	Mat V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
 
	V1 = H * V2;
	//左上角(0,0,1)
	cout << "V2: " << V2 << endl;
	cout << "V1: " << V1 << endl;
	corners.left_top.x = v1[0] / v1[2];
	corners.left_top.y = v1[1] / v1[2];
 
	//左下角(0,src.rows,1)
	v2[0] = 0;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.left_bottom.x = v1[0] / v1[2];
	corners.left_bottom.y = v1[1] / v1[2];
 
	//右上角(src.cols,0,1)
	v2[0] = src.cols;
	v2[1] = 0;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.right_top.x = v1[0] / v1[2];
	corners.right_top.y = v1[1] / v1[2];
 
	//右下角(src.cols,src.rows,1)
	v2[0] = src.cols;
	v2[1] = src.rows;
	v2[2] = 1;
	V2 = Mat(3, 1, CV_64FC1, v2);  //列向量
	V1 = Mat(3, 1, CV_64FC1, v1);  //列向量
	V1 = H * V2;
	corners.right_bottom.x = v1[0] / v1[2];
	corners.right_bottom.y = v1[1] / v1[2];
 
}

//优化两图的连接处，使得拼接自然
void OptimizeSeam(Mat& img1, Mat& trans, Mat& dst, four_corners_t& corners)
{
	int start = MIN(corners.left_top.x, corners.left_bottom.x);//开始位置，即重叠区域的左边界  
 
	double processWidth = img1.cols - start;//重叠区域的宽度  
	int rows = dst.rows;
	int cols = img1.cols; //注意，是列数*通道数
	double alpha = 1;//img1中像素的权重  
	for (int i = 0; i < rows; i++)
	{
		uchar* p = img1.ptr<uchar>(i);  //获取第i行的首地址
		uchar* t = trans.ptr<uchar>(i);
		uchar* d = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			//如果遇到图像trans中无像素的黑点，则完全拷贝img1中的数据
			if (t[j * 3] == 0 && t[j * 3 + 1] == 0 && t[j * 3 + 2] == 0)
			{
				alpha = 1;
			}
			else
			{
				//img1中像素的权重，与当前处理点距重叠区域左边界的距离成正比，实验证明，这种方法确实好  
				alpha = (processWidth - (j - start)) / processWidth;
			}
 
			d[j * 3] = p[j * 3] * alpha + t[j * 3] * (1 - alpha);
			d[j * 3 + 1] = p[j * 3 + 1] * alpha + t[j * 3 + 1] * (1 - alpha);
			d[j * 3 + 2] = p[j * 3 + 2] * alpha + t[j * 3 + 2] * (1 - alpha);
 
		}
	}
 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_stitcher");
  ros::NodeHandle n;
  //用之前声明的节点句柄初始化it，其实这里的it和nh的功能基本一样，可以像之前一样使用it来发布和订阅相消息。
  image_transport::ImageTransport it(n);
  image_transport::Publisher pub_ground = it.advertise("img_stitch/img_ground", 1);
  image_transport::Publisher pub_car = it.advertise("img_stitch/img_car", 1);

  VideoCapture cap1(2);//right
  VideoCapture cap2(1);//left

  bool run_flag = true;
  Mat img_left;
  Mat img_right;
  Mat img_result;

  if (cap1.isOpened() && cap2.isOpened())
	{
		cout << "------" << endl;
		cout << "Camera Initialization Success！\n" << endl;
	}
	else
	{
		cout << "------" << endl;
		cout << "Camera Initialization FAILED!" << endl;
		cout << "Aborting..." << endl;
		return -1;
	}
  cap1.set(CV_CAP_PROP_FRAME_WIDTH, 960);
	cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 540);
	cap2.set(CV_CAP_PROP_FRAME_WIDTH, 960);
	cap2.set(CV_CAP_PROP_FRAME_HEIGHT, 540);

  Mat initial_frame_1, initial_frame_2;
  cap1 >> initial_frame_1;//right
  cap2 >> initial_frame_2;//left
//   imwrite("right.jpg",initial_frame_1);
//   imwrite("left.jpg",initial_frame_2);


  Ptr<SURF> surf;            //创建方式和OpenCV2中的不一样,并且要加上命名空间xfreatures2d
							   //否则即使配置好了还是显示SURF为未声明的标识符  
	surf = SURF::create(800);
 
	BFMatcher matcher;         //实例化一个暴力匹配器
	Mat c, d;
	vector<KeyPoint>key1, key2;
	vector<DMatch> matches;    //DMatch是用来描述匹配好的一对特征点的类，包含这两个点之间的相关信息
							   //比如左图有个特征m，它和右图的特征点n最匹配，这个DMatch就记录它俩最匹配，并且还记录m和n的
							   //特征向量的距离和其他信息，这个距离在后面用来做筛选
 
	surf->detectAndCompute(initial_frame_1, Mat(), key1, c);//输入图像，输入掩码，输入特征点，输出Mat，存放所有特征点的描述向量
	surf->detectAndCompute(initial_frame_2, Mat(), key2, d);//这个Mat行数为特征点的个数，列数为每个特征向量的尺寸，SURF是64（维）
 
	matcher.match(d, c, matches);             //匹配，数据来源是特征向量，结果存放在DMatch类型里面  
 
											  //sort函数对数据进行升序排列
	sort(matches.begin(), matches.end());     //筛选匹配点，根据match里面特征对的距离从小到大排序
	vector< DMatch > good_matches;
	int ptsPairs = std::min(50, (int)(matches.size() * 0.15));
	cout << ptsPairs << endl;
	for (int i = 0; i < ptsPairs; i++)
	{
		good_matches.push_back(matches[i]);//距离最小的50个压入新的DMatch
	}
	Mat outimg;                                //drawMatches这个函数直接画出摆在一起的图
	drawMatches(initial_frame_2, key2, initial_frame_1, key1, good_matches, outimg, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  //绘制匹配点  
 
 
	// imshow("desktop", outimg);
 
	///////////////////////图像配准及融合////////////////////////
 
	vector<Point2f> imagePoints1, imagePoints2;
 
	for (int i = 0; i<good_matches.size(); i++)
	{
		imagePoints2.push_back(key2[good_matches[i].queryIdx].pt);
		imagePoints1.push_back(key1[good_matches[i].trainIdx].pt);
	}
 
	//获取图像1到图像2的投影映射矩阵 尺寸为3*3  
	Mat homo = findHomography(imagePoints1, imagePoints2, CV_RANSAC);
	Mat homo_ground = (Mat_<double>(3,3)<<1.086467751533472, 0.04833123972395902, 388.8406235038879,
 						0.009083400035754726, 1.044417309922175, -21.19354755806051,
 						9.955491555686608e-05, 2.538114036079827e-05, 1);

	Mat homo_car = (Mat_<double>(3,3)<<1.082947263115328, 0.0545396038988664, 463.4449670881162,
 					0.002573333206183431, 1.041444226990858, -20.09008386538618,
					 6.084102207561295e-05, 4.887458110802634e-05, 1);

	// Mat homo = (Mat_<double>(3,3)<<1.577654034920638, -0.166779285620344, -315.5858533696461, 0.1976952662635193, 1.364393154022443, -176.0950934920215, 0.0005565288039895187, -0.0001611685131212428, 1);
	////也可以使用getPerspectiveTransform方法获得透视变换矩阵，不过要求只能有4个点，效果稍差  
	//Mat homo=getPerspectiveTransform(imagePoints1,imagePoints2);  
	// cout << "变换矩阵为：\n" << homo << endl << endl; //输出映射矩阵   
 
												//计算配准图的四个顶点坐标
	CalcCorners(homo_ground, initial_frame_1, corners_ground);
	CalcCorners(homo_car, initial_frame_1, corners_car);
	// cout << "left_top:" << corners.left_top << endl;
	// cout << "left_bottom:" << corners.left_bottom << endl;
	// cout << "right_top:" << corners.right_top << endl;
	// cout << "right_bottom:" << corners.right_bottom << endl;
 
												//图像配准  
	Mat imageTransformGround, imageTransformCar;
	warpPerspective(initial_frame_1, imageTransformGround, homo_ground, Size(MAX(corners_ground.right_top.x, corners_ground.right_bottom.x), initial_frame_2.rows));
	warpPerspective(initial_frame_1, imageTransformCar, homo_car, Size(MAX(corners_car.right_top.x, corners_car.right_bottom.x), initial_frame_2.rows));
	// imshow("warpPerspective", imageTransformGround);

  waitKey(100);
  double t;

  while (run_flag)
	{
      if (cap1.read(img_right) && cap2.read(img_left))
      {
        t = getTickCount();
        warpPerspective(img_right, imageTransformGround, homo_ground, Size(MAX(corners_ground.right_top.x, corners_ground.right_bottom.x), initial_frame_2.rows));
        warpPerspective(img_right, imageTransformCar, homo_car, Size(MAX(corners_car.right_top.x, corners_car.right_bottom.x), initial_frame_2.rows));

        int dst_width_ground = imageTransformGround.cols;  //取最右点的长度为拼接图的长度
        int dst_height_ground = img_left.rows;
        int dst_width_car = imageTransformCar.cols;  //取最右点的长度为拼接图的长度
        int dst_height_car = img_left.rows;
      
        Mat dst_ground(dst_height_ground, dst_width_ground, CV_8UC3);
        dst_ground.setTo(0);
        Mat dst_car(dst_height_car, dst_width_car, CV_8UC3);
        dst_car.setTo(0);
      
        imageTransformGround.copyTo(dst_ground(Rect(0, 0, imageTransformGround.cols, imageTransformGround.rows)));
        img_left.copyTo(dst_ground(Rect(0, 0, img_left.cols, img_left.rows)));
        imageTransformCar.copyTo(dst_car(Rect(0, 0, imageTransformCar.cols, imageTransformCar.rows)));
        img_left.copyTo(dst_car(Rect(0, 0, img_left.cols, img_left.rows)));
      
        // imshow("direct_stitch", dst_ground);

        OptimizeSeam(img_left, imageTransformGround, dst_ground, corners_ground);
        OptimizeSeam(img_left, imageTransformCar, dst_car, corners_car);
        t = ((double)getTickCount() - t) / getTickFrequency();
		cout << "RunningTime: " << t << endl;
        // imshow("ground", dst_ground);
        // imshow("car", dst_car);
		auto rects = findParkingSpace(dst_ground);
		for(const auto& rect:rects)
		{
			cv::rectangle(dst_ground,rect,{255,0,0},2);
		}
		imshow("ParkingSpace", dst_ground);
        waitKey(5);
        //图像格式转换
        sensor_msgs::ImagePtr msg_ground = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst_ground).toImageMsg();
        sensor_msgs::ImagePtr msg_car = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst_car).toImageMsg();
        pub_ground.publish(msg_ground);
        pub_car.publish(msg_car);
        ros::spinOnce();
		// imshow("img_left", img_left);
		// imshow("img_right", img_right);

		if(waitKey(10) == 'q') run_flag = false;
      }

  }
  cap1.release();
  cap2.release();


  return 0;
}
