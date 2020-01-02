#include <ros/ros.h>
#include <string>
#include <serial/serial.h>
#include <unistd.h>
#include <math.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
// #include <control.h>


#define PI 3.1415926535898
geometry_msgs::Pose VehiclePose;
geometry_msgs::Pose ParklotPose;
std::vector<geometry_msgs::Pose> vParklotPose;
std::vector<int> Angle;
std::vector<int> Velocity;
std::vector<size_t> RunTime;
double lk; //前视距离系数
double Lfc; // 前视距离
double L;//车辆轴距/m
struct vehicle_state
{
    double x;
    double y;
    double yaw;
    double v;
};
struct delta_and_index
{
    double delta;
    int index;
};


int serial_write(serial::Serial &ser, std::string &serial_msg);
int serial_read(serial::Serial &ser, std::string &result);
int control_msg_send(serial::Serial &serial, int &angle, int &velocity, size_t &run_time);
int StateUpdate(vehicle_state &CarState);
int CalcTargetIndex(vehicle_state CarState, std::vector<double> cx, std::vector<double> cy);
delta_and_index PurePursuitControl(vehicle_state CarState, std::vector<double> cx, std::vector<double> cy, int target_index);
geometry_msgs::Vector3 euler(geometry_msgs::Quaternion q);
void parking_plan(geometry_msgs::Pose &vehicle_pose, geometry_msgs::Pose &parklot_pose, std::vector<int> &angle, std::vector<int> &velocity, std::vector<size_t> &run_time);
void PoseInfoCallback(const nav_msgs::Odometry &msg);

int serial_write(serial::Serial &ser, std::string &serial_msg)
{
	ser.write(serial_msg);
	return 0;
}

int serial_read(serial::Serial &ser, std::string &result)
{
	result = ser.read(ser.available()); 
	return 0;
}

int control_msg_send(serial::Serial &serial, int &angle, int &velocity, size_t &run_time)
{
    std::vector<uint8_t> msghead(1, 0xc5);
    std::vector<uint8_t> msgtail(1, 0x00);
    std::stringstream angle_s;
    std::stringstream velocity_s;
    angle_s << angle;
    velocity_s << velocity;
    std::string msg =  angle_s.str() + " " + velocity_s.str();
    std::cout << msg << std::endl;	
    std::cout << "angle output: " << angle << "  " << "velocity output: " << velocity << "  " << "run time: " << run_time << std::endl;
    if(serial.available())
    {
        serial.write(msghead);
        serial.write(msg);
        serial.write(msgtail);
    }
    usleep(run_time);
    
    return 0;
}

int StateUpdate(vehicle_state &CarState)
{
    //接受Apriltag检测结果
    ros::spinOnce();
    //计算状态
    geometry_msgs::Vector3 vehicle_orientation = euler(VehiclePose.orientation);
    CarState.x = VehiclePose.position.x;
    CarState.y = VehiclePose.position.y;
    CarState.yaw = vehicle_orientation.z;
    CarState.v = 0.05;//m/s
}

int CalcTargetIndex(vehicle_state CarState, std::vector<double> cx, std::vector<double> cy)
{
    std::vector<double> dx;
    std::vector<double> dy;
    std::vector<double> d;
    double num_compare = 10000;
    int min_index = 0;
    double L;
    double dx2, dy2;
    for (size_t i = 0; i < cx.size(); i++)
    {
        dx.push_back(CarState.x - cx[i]);
        dy.push_back(CarState.y - cy[i]);
        d.push_back(sqrt((CarState.x - cx[i])*(CarState.x - cx[i]) + (CarState.y - cy[i])*(CarState.y - cy[i])));
    }
    for (size_t i = 0; i < d.size(); i++)
    {
        if(d[i] < num_compare)
        {
            num_compare = d[i];
            min_index = i;
        }
    }
    L = 0;
    double Lf = lk * CarState.v + Lfc;
    while ((Lf > L) && ((min_index + 1) < cx.size()))
    {
        dx2 = cx[min_index + 1] - cx[min_index];
        dy2 = cy[min_index + 1] - cy[min_index];
        L += sqrt(dx2*dx2 + dy2*dy2);
        min_index ++;
    }
    return min_index;
}

delta_and_index PurePursuitControl(vehicle_state CarState, std::vector<double> cx, std::vector<double> cy, int target_index)
{
    double tx, ty, alpha, delta;
    int index = CalcTargetIndex(CarState, cx, cy);
    if(index <= target_index)
    {
        index = target_index;
    }
    if(index < cx.size())
    {
        tx = cx[index];
        ty = cy[index];
    }else
    {
        index = cx.size() - 1;
        tx = cx[index];
        ty = cy[index];
    }
    alpha = atan2(ty - CarState.y, tx - CarState.x) - CarState.yaw;
    if(CarState.v < 0) alpha = PI - alpha;
    double Lf = lk*CarState.v + Lfc;
    delta = atan2(2*L*sin(alpha)/Lf, 1.0);
    delta_and_index result;
    result.delta = delta;
    result.index = index;
    return result;
    
}

//pmx - 将四元数转换为欧拉角
geometry_msgs::Vector3 euler(geometry_msgs::Quaternion q)
{
    geometry_msgs::Vector3 eu;
    eu.x = atan2(2 * (q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    eu.y = asin(-2 * (q.x*q.z - q.w*q.y));
    eu.z = atan2(2 * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    return eu;
}

void parking_plan(geometry_msgs::Pose &vehicle_pose, geometry_msgs::Pose &parklot_pose, std::vector<int> &angle, std::vector<int> &velocity, std::vector<size_t> &run_time)
{
    geometry_msgs::Vector3 vehicle_orientation = euler(vehicle_pose.orientation);
    // std::cout << "x: " << vehicle_orientation.x << " " << "y: " << vehicle_orientation.y << " " << "z: " << vehicle_orientation.z << std::endl;
    std::cout <<  "Initial angle: " << vehicle_orientation.z << std::endl;
    float init_angle = vehicle_orientation.z;
    float pix_per_cm = 0.01;

    //stage 1
    if(init_angle > 0)
    {
        angle.push_back(45);
    }else
    {
        angle.push_back(-45);
    }
    velocity.push_back(200);
    run_time.push_back(size_t(std::abs(init_angle*1000000*5)));

    angle.push_back(0);
    velocity.push_back(0);
    run_time.push_back(1000000);

    //stage 2
    angle.push_back(0);
    velocity.push_back(200);
    float distance2 = parklot_pose.position.x + 20 * pix_per_cm - vehicle_pose.position.x - 20 * pix_per_cm * std::abs(sin(init_angle));
    run_time.push_back(size_t(distance2/20/pix_per_cm*1000000*3));

    angle.push_back(0);
    velocity.push_back(0);
    run_time.push_back(1000000);

    //stage 3
    angle.push_back(-45);
    velocity.push_back(-200);
    run_time.push_back(size_t(0.7854*1000000*5));

    angle.push_back(0);
    velocity.push_back(0);
    run_time.push_back(1000000);

    //stage 4
    angle.push_back(0);
    velocity.push_back(-200);
    float distance4 = parklot_pose.position.y - vehicle_pose.position.y + 20 * pix_per_cm * (1 - cos(init_angle));
    run_time.push_back(size_t(distance4/20/pix_per_cm*1000000*3));

    angle.push_back(0);
    velocity.push_back(0);
    run_time.push_back(1000000);

}

//Callback functuion to subscribe vehicle's initial pose and target parklot's pose
void PoseInfoCallback(const nav_msgs::Odometry &msg)
{
    VehiclePose.position.x = msg.pose.pose.position.x;
    VehiclePose.position.y = msg.pose.pose.position.y;
    VehiclePose.position.z = msg.pose.pose.position.z;
    VehiclePose.orientation.x = msg.pose.pose.orientation.x;
    VehiclePose.orientation.y = msg.pose.pose.orientation.y;
    VehiclePose.orientation.z = msg.pose.pose.orientation.z;
    VehiclePose.orientation.w = msg.pose.pose.orientation.w;
    std::cout << "Vehicle Pose detected." << std::endl;
}

int main(int argc, char **argv)
{
    ParklotPose.position.x = 0.6;
    ParklotPose.position.y = 0.8;
    vParklotPose.push_back(ParklotPose);
    ParklotPose.position.x = 1.08;
    ParklotPose.position.y = 0.86;
    vParklotPose.push_back(ParklotPose);
    ParklotPose.position.x = 1.45;
    ParklotPose.position.y = 0.86;
    vParklotPose.push_back(ParklotPose);
    ParklotPose.position.x = 1.82;
    ParklotPose.position.y = 0.86;
    vParklotPose.push_back(ParklotPose);
    ros::init(argc, argv, "control_message");
    ros::NodeHandle nh;
    ros::Subscriber PoseInfoSub = nh.subscribe("tag_Odometry", 100, PoseInfoCallback);    
	serial::Serial ser;
	//初始化串口（蓝牙）
	try 
    { 
        ser.setPort("/dev/ttyUSB0"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e)
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
    
    if(ser.isOpen())
    { 
        ROS_INFO_STREAM("Serial Port initialized."); 
    } 
    else 
    { 
        return -1; 
    } 
    //清空控制参数
    Angle.clear();
    Velocity.clear();
    RunTime.clear();
    std::cout << "Initialing..." << std::endl;
    usleep(5000000);
    //接受Apriltag检测结果
    ros::spinOnce();
    // 计算控制参数
    parking_plan(VehiclePose, vParklotPose[2], Angle, Velocity, RunTime);
    // 执行控制
    // std::cout<< "angle size : "<< Angle.size() <<std::endl;
    for(size_t i = 0; i < Angle.size(); i++)
    {
        control_msg_send(ser, Angle[i], Velocity[i], RunTime[i]);
    }

    std::cout << "Done." << std::endl;
	return 0;

}
