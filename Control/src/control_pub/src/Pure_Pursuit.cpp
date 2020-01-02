#include <iostream>
#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <serial/serial.h>

/*install some package, command below:
$ sudo apt-get ros-kinetic-navigation* 
*/

#define Forward true
#define Reverse false
/********************/
/* CLASS DEFINITION */
/********************/
class PurePursuit
{
    public:
        PurePursuit();
        void initPath();
        void initParkingPath(int& patklot_index);
        bool isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isReverseWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose);
        bool isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos);
        double getYawFromPose(const geometry_msgs::Pose& carPose);        
        double getEta(const geometry_msgs::Pose& carPose);
        double getCar2GoalDist();
        double getSteering(double eta);
        geometry_msgs::Point get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose);
        int serial_write(serial::Serial &ser, std::string &serial_msg);
        int serial_read(serial::Serial &ser, std::string &result);
        int control_msg_send(serial::Serial &serial, double &angle, double &velocity);

    private:
        ros::NodeHandle n_;
        ros::Subscriber odom_sub, path_sub, goal_sub;
        ros::Publisher cmdvel_pub;
        ros::Timer timer1, timer2;
        tf::TransformListener tf_listener;

        geometry_msgs::Point odom_goal_pos, goal_pos;
        geometry_msgs::Twist cmd_vel;
        nav_msgs::Odometry odom;
        nav_msgs::Path map_path, odom_path, map_path_forward, map_path_reverse;

        serial::Serial ser;

        double L, Lfw, Lrv, Vcmd, lfw, lrv, steering, velocity;
        double steering_gain, base_angle, goal_radius, speed_incremental;
        int controller_freq;
        bool foundForwardPt, goal_received, goal_reached, cmd_vel_mode, debug_mode, smooth_accel, driving_direction;

        void odomCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCallBack(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void controlLoopCallBack(const ros::TimerEvent&);

}; // end of class


PurePursuit::PurePursuit()
{
    //Private parameters handler
    ros::NodeHandle pn("~");

    //Car parameter
    pn.param("L", L, 0.15); // length of car
    pn.param("Vcmd", Vcmd, 100.0);// reference speed (the unit should be m/s, however it is not.)
    pn.param("Lfw", Lfw, 0.5); // forward look ahead distance (m)
    pn.param("lfw", lfw, 0.08); // distance between front and the center of car
    pn.param("Lrv", Lrv, 10.0);
    pn.param("lrv", lrv, 10.0);

    //Controller parameter
    pn.param("controller_freq", controller_freq, 20);
    pn.param("steering_gain", steering_gain, -100.0);
    pn.param("goal_radius", goal_radius, 0.05); // goal radius (m)
    pn.param("base_angle", base_angle, 0.0); // neutral point of servo (rad) 
    pn.param("cmd_vel_mode", cmd_vel_mode, false); // whether or not publishing cmd_vel
    pn.param("debug_mode", debug_mode, false); // debug mode
    pn.param("smooth_accel", smooth_accel, false); // smooth the acceleration of car
    pn.param("speed_incremental", speed_incremental, 30.0); // speed incremental value (discrete acceleraton), unit: m/s

    //Publishers and Subscribers
    odom_sub = n_.subscribe("/odom", 1, &PurePursuit::odomCallBack, this);
    path_sub = n_.subscribe("/pure_pursuit/global_planner", 1, &PurePursuit::pathCallBack, this);
    goal_sub = n_.subscribe("/pure_pursuit/goal", 1, &PurePursuit::goalCallBack, this);
    if(cmd_vel_mode) cmdvel_pub = n_.advertise<geometry_msgs::Twist>("/pure_pursuit/cmd_vel", 1);    

    //Timer
    timer1 = n_.createTimer(ros::Duration((1.0)/controller_freq), &PurePursuit::controlLoopCallBack, this); // Duration(0.05) -> 20Hz


    //Init variables
    foundForwardPt = false;
    goal_received = false;
    goal_reached = false;
    velocity = 0.0;
    steering = base_angle;
    driving_direction = Forward;

    //Show info
    ROS_INFO("[param] base_angle: %f", base_angle);
    ROS_INFO("[param] Vcmd: %f", Vcmd);
    ROS_INFO("[param] Lfw: %f", Lfw);

    // initPath();
    std::cout << "++++++++++++++++++++++++++++++++++++++++" << std::endl;
    std::cout << "+ Choose a parklot...[options:0,1,2,3] +" << std::endl;
    std::cout << "++++++++++++++++++++++++++++++++++++++++" << std::endl;
    int input;
    std::cin >> input;
    initParkingPath(input);

    cmd_vel = geometry_msgs::Twist();

    //initialize bluetooth
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
        ROS_ERROR_STREAM("Error(s), you need to restart the program"); 
        // return -1; 
    } 
    
    if(ser.isOpen())
    { 
        ROS_INFO_STREAM("Serial Port initialized."); 
    } 
    else 
    { 
        ROS_ERROR_STREAM("Error(s), you need to restart the program"); 
        // return -1; 
    } 
}

void PurePursuit::initPath()
{
    map_path.header.frame_id = "map";
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = 0.2;
    this_pose_stamped.pose.position.y = 0.1;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    // this_pose_stamped.header.stamp=current_time;
    // this_pose_stamped.header.frame_id="map";
    for(int i = 0; i < 9; i++)
    {
        this_pose_stamped.pose.position.y += 0.1;
        map_path.poses.push_back(this_pose_stamped);
    }
    // map_path.poses.push_back(this_pose_stamped);
    goal_pos.x = 0.2;
    goal_pos.y = 1.0;
    goal_received = true;
}

void PurePursuit::initParkingPath(int& patklot_index)
{
    if(patklot_index == 1)
    {
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = 0.2;
        this_pose_stamped.pose.position.y = 0.2;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        // this_pose_stamped.header.stamp=current_time;
        // this_pose_stamped.header.frame_id="map";
        for(int i = 0; i < 24; i++)
        {
            this_pose_stamped.pose.position.x += 0.05;
            map_path_forward.poses.push_back(this_pose_stamped);
        }
        for(int i = 0; i < 16; i += 2)
        {
            this_pose_stamped.pose.position.x = 1.28 - sin(((float)i)/10);
            this_pose_stamped.pose.position.y = 0.4 - cos(((float)i)/10);
            map_path_reverse.poses.push_back(this_pose_stamped);
        }
        for(int i = 0; i < 9; i++)
        {
            this_pose_stamped.pose.position.y += 0.05;
            map_path_reverse.poses.push_back(this_pose_stamped);
        }
    }
    if(patklot_index == 2)
    {
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = 0.2;
        this_pose_stamped.pose.position.y = 0.2;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        // this_pose_stamped.header.stamp=current_time;
        // this_pose_stamped.header.frame_id="map";
        for(int i = 0; i < 34; i++)
        {
            this_pose_stamped.pose.position.x += 0.05;
            map_path_forward.poses.push_back(this_pose_stamped);
        }
        for(int i = 0; i < 16; i += 2)
        {
            this_pose_stamped.pose.position.x = 1.65 - sin(((float)i)/10);
            this_pose_stamped.pose.position.y = 0.4 - cos(((float)i)/10);
            map_path_reverse.poses.push_back(this_pose_stamped);
        }
        for(int i = 0; i < 9; i++)
        {
            this_pose_stamped.pose.position.y += 0.05;
            map_path_reverse.poses.push_back(this_pose_stamped);
        }
    }
    if(patklot_index == 3)
    {
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = 0.2;
        this_pose_stamped.pose.position.y = 0.2;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(0);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        // this_pose_stamped.header.stamp=current_time;
        // this_pose_stamped.header.frame_id="map";
        for(int i = 0; i < 41; i++)
        {
            this_pose_stamped.pose.position.x += 0.05;
            map_path_forward.poses.push_back(this_pose_stamped);
        }
        for(int i = 0; i < 16; i += 2)
        {
            this_pose_stamped.pose.position.x = 2.02 - sin(((float)i)/10);
            this_pose_stamped.pose.position.y = 0.4 - cos(((float)i)/10);
            map_path_reverse.poses.push_back(this_pose_stamped);
        }
        for(int i = 0; i < 9; i++)
        {
            this_pose_stamped.pose.position.y += 0.05;
            map_path_reverse.poses.push_back(this_pose_stamped);
        }
    }
    // map_path.poses.push_back(this_pose_stamped);
    goal_pos = map_path_forward.poses[map_path_forward.poses.size() - 1].pose.position;
    goal_received = true;
}


void PurePursuit::odomCallBack(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    this->odom = *odomMsg;
}


void PurePursuit::pathCallBack(const nav_msgs::Path::ConstPtr& pathMsg)
{
    this->map_path = *pathMsg;
}


// CallBack: Update goal status
void PurePursuit::goalCallBack(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    this->goal_pos = goalMsg->pose.position;    
    try
    {
        geometry_msgs::PoseStamped odom_goal;
        tf_listener.transformPose("odom", ros::Time(0) , *goalMsg, "map" ,odom_goal);
        odom_goal_pos = odom_goal.pose.position;
        goal_received = true;
        goal_reached = false;
    }
    catch(tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

//从车辆姿态中获取yaw值
double PurePursuit::getYawFromPose(const geometry_msgs::Pose& carPose)
{
    float x = carPose.orientation.x;
    float y = carPose.orientation.y;
    float z = carPose.orientation.z;
    float w = carPose.orientation.w;

    double tmp,yaw;
    tf::Quaternion q(x,y,z,w);
    tf::Matrix3x3 quaternion(q);
    quaternion.getRPY(tmp,tmp, yaw);

    return yaw;
}

//判断是否为前向点
bool PurePursuit::isForwardWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x >0) /*is Forward WayPt*/
        return true;
    else
        return false;
}

//判断是否为后向点
bool PurePursuit::isReverseWayPt(const geometry_msgs::Point& wayPt, const geometry_msgs::Pose& carPose)
{
    float car2wayPt_x = wayPt.x - carPose.position.x;
    float car2wayPt_y = wayPt.y - carPose.position.y;
    double car_theta = getYawFromPose(carPose);

    float car_car2wayPt_x = cos(car_theta)*car2wayPt_x + sin(car_theta)*car2wayPt_y;
    float car_car2wayPt_y = -sin(car_theta)*car2wayPt_x + cos(car_theta)*car2wayPt_y;

    if(car_car2wayPt_x < 0) /*is Reverse WayPt*/
        return true;
    else
        return false;
}

//判断车到目标点的距离是否小于设定的前视距离
bool PurePursuit::isWayPtAwayFromLfwDist(const geometry_msgs::Point& wayPt, const geometry_msgs::Point& car_pos)
{
    double dx = wayPt.x - car_pos.x;
    double dy = wayPt.y - car_pos.y;
    double dist = sqrt(dx*dx + dy*dy);

    if(dist < Lfw)
        return false;
    else if(dist >= Lfw)
        return true;
}

//获得车的当前位置与下一个目标点的向量
geometry_msgs::Point PurePursuit::get_odom_car2WayPtVec(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point carPose_pos = carPose.position;
    double carPose_yaw = getYawFromPose(carPose);
    geometry_msgs::Point forwardPt;
    geometry_msgs::Point odom_car2WayPtVec;
    foundForwardPt = false;

    if(!goal_reached){
        if(driving_direction) // Forward
        {
            for(int i =0; i< map_path_forward.poses.size(); i++)
            {
                geometry_msgs::PoseStamped map_path_pose = map_path_forward.poses[i];
                geometry_msgs::PoseStamped odom_path_pose;

                try
                {
                    odom_path_pose = map_path_pose;
                    geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                    bool _isForwardWayPt = isForwardWayPt(odom_path_wayPt,carPose);

                    if(_isForwardWayPt)
                    {
                        bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                        if(_isWayPtAwayFromLfwDist)
                        {
                            forwardPt = odom_path_wayPt;
                            foundForwardPt = true;
                            break;
                        }
                    }
                }
                catch(tf::TransformException &ex)
                {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
            }
        }else //Reverse
        {
            for(int i =0; i< map_path_reverse.poses.size(); i++)
            {
                geometry_msgs::PoseStamped map_path_pose = map_path_reverse.poses[i];
                geometry_msgs::PoseStamped odom_path_pose;

                try
                {
                    odom_path_pose = map_path_pose;
                    geometry_msgs::Point odom_path_wayPt = odom_path_pose.pose.position;
                    bool _isReverseWayPt = isReverseWayPt(odom_path_wayPt,carPose);

                    if(_isReverseWayPt)
                    {
                        bool _isWayPtAwayFromLfwDist = isWayPtAwayFromLfwDist(odom_path_wayPt,carPose_pos);
                        if(_isWayPtAwayFromLfwDist)
                        {
                            forwardPt = odom_path_wayPt;
                            foundForwardPt = true;
                            break;
                        }
                    }
                }
                catch(tf::TransformException &ex)
                {
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(1.0).sleep();
                }
            }
        }
        
        
        
    }
    else if(goal_reached)
    {
        forwardPt = odom_goal_pos;
        foundForwardPt = false;
        //ROS_INFO("goal REACHED!");
    }
    
    if(driving_direction) // Forward
    {
        odom_car2WayPtVec.x = cos(carPose_yaw)*(forwardPt.x - carPose_pos.x) + sin(carPose_yaw)*(forwardPt.y - carPose_pos.y);
        odom_car2WayPtVec.y = -sin(carPose_yaw)*(forwardPt.x - carPose_pos.x) + cos(carPose_yaw)*(forwardPt.y - carPose_pos.y);
    }else //Reverse
    {
        double carPose_x = carPose_pos.x - 2*this->lfw*cos(carPose_yaw);
        double carPose_y = carPose_pos.y - 2*this->lfw*sin(carPose_yaw);
        odom_car2WayPtVec.x = cos(carPose_yaw)*(carPose_x - forwardPt.x) + sin(carPose_yaw)*(carPose_y - forwardPt.y);
        odom_car2WayPtVec.y = -sin(carPose_yaw)*(carPose_x - forwardPt.x) + cos(carPose_yaw)*(carPose_y - forwardPt.y);
    }

    return odom_car2WayPtVec;
}

//获得转向角
double PurePursuit::getEta(const geometry_msgs::Pose& carPose)
{
    geometry_msgs::Point odom_car2WayPtVec = get_odom_car2WayPtVec(carPose);
    return atan2(odom_car2WayPtVec.y,odom_car2WayPtVec.x);
}


double PurePursuit::getCar2GoalDist()
{
    geometry_msgs::Point car_pose = this->odom.pose.pose.position;
    double car2goal_x = this->odom_goal_pos.x - car_pose.x;
    double car2goal_y = this->odom_goal_pos.y - car_pose.y;

    return sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
}


double PurePursuit::getSteering(double eta)
{
    return atan2((this->L*sin(eta)),(this->Lfw/2 + this->lfw*cos(eta)));
}


// Callback: Check if the car is inside the goal area or not 

int PurePursuit::serial_write(serial::Serial &ser, std::string &serial_msg)
{
	ser.write(serial_msg);
	return 0;
}

int PurePursuit::serial_read(serial::Serial &ser, std::string &result)
{
	result = ser.read(ser.available()); 
	return 0;
}

int PurePursuit::control_msg_send(serial::Serial &serial, double &angle, double &velocity)
{
    std::vector<uint8_t> msghead(1, 0xc5);
    std::vector<uint8_t> msgtail(1, 0x00);
    int angle1 = (int)angle;
    int velocity1 = (int)velocity;
    std::stringstream angle_s;
    std::stringstream velocity_s;
    // angle_s << angle;
    // velocity_s << velocity;
    angle_s << angle1;
    velocity_s << velocity1;
    std::string msg =  angle_s.str() + " " + velocity_s.str();
    // std::cout << msg << std::endl;	
    // std::cout << "angle output: " << angle << "  " << "velocity output: " << velocity << std::endl;
    if(serial.available())
    {
        serial.write(msghead);
        serial.write(msg);
        serial.write(msgtail);
    }    
    return 0;
}

// Timer: Control Loop
void PurePursuit::controlLoopCallBack(const ros::TimerEvent&)
{

    geometry_msgs::Pose carPose = this->odom.pose.pose;
    geometry_msgs::Twist carVel = this->odom.twist.twist;

    if(this->goal_received)
    {
        /*Estimate Steering Angle*/
        double eta = getEta(carPose);  
        if(foundForwardPt)
        {
            this->steering = this->base_angle + getSteering(eta)*this->steering_gain;
            // if(!driving_direction) this->steering = -this->steering;

            /*Estimate Gas Input*/
            if(!this->goal_reached)
            {
                if(this->smooth_accel) 
                    this->velocity = std::min(this->velocity + this->speed_incremental, this->Vcmd);
                else
                    this->velocity = this->Vcmd;
                if(!driving_direction) this->velocity = -this->velocity;
                if(debug_mode) ROS_INFO("Velocity = %.2f, Steering = %.2f", this->velocity, this->steering);
            }
        }
        double car2goal_x = this->goal_pos.x - carPose.position.x;
        double car2goal_y = this->goal_pos.y - carPose.position.y;
        double dist2goal = sqrt(car2goal_x*car2goal_x + car2goal_y*car2goal_y);
        if(dist2goal < this->goal_radius)
        {
            this->goal_reached = true;
            this->goal_received = false;
            ROS_INFO("Goal Reached !");
        }
    }

    if(this->goal_reached)
    {
        this->velocity = 0.0;
        this->steering = this->base_angle;
        if(driving_direction)
        {
            this->goal_pos = map_path_reverse.poses[map_path_reverse.poses.size() - 1].pose.position;
            this->goal_reached = false;
            this->goal_received = true;
            driving_direction = Reverse;
        }
    }
    

    control_msg_send(ser, this->steering, this->velocity);
    std::cout << " goal_received: " << this->goal_received << " goal_reached: " << this->goal_reached << " steering: " << this->steering << " velocity: " << this->velocity << std::endl;

    if(this->cmd_vel_mode)
    {
        this->cmd_vel.linear.x = this->velocity;
        this->cmd_vel.angular.z = this->steering;
        this->cmdvel_pub.publish(this->cmd_vel);
    }   
}


/*****************/
/* MAIN FUNCTION */
/*****************/
int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "PurePursuit");
    PurePursuit controller;
    ros::AsyncSpinner spinner(2); // Use multi threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
