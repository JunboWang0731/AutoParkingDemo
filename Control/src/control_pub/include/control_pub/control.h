// #ifndef CONTROL_H
// #define CONTROL_H

// #include <ros/ros.h>
// #include <string>
// #include <serial/serial.h>
// #include <unistd.h>
// #include <math.h>
// #include <geometry_msgs/Pose.h>
// #include <geometry_msgs/Vector3.h>
// #include <geometry_msgs/Quaternion.h>
// #include <nav_msgs/Odometry.h>

// struct vehicle_state
// {
//     double x;
//     double y;
//     double yaw;
//     double v;
// };

// int serial_write(serial::Serial &ser, std::string &serial_msg);
// int serial_read(serial::Serial &ser, std::string &result);
// int control_msg_send(serial::Serial &serial, int &angle, int &velocity, size_t &run_time);
// int StateUpdate(vehicle_state &CarState);
// int CalcTargetIndex(vehicle_state CarState, std::vector<double> cx, std::vector<double> cy);
// int PurePursuitControl(vehicle_state CarState, std::vector<double> cx, std::vector<double> cy, int target_index);
// geometry_msgs::Vector3 euler(geometry_msgs::Quaternion q);
// void parking_plan(geometry_msgs::Pose &vehicle_pose, geometry_msgs::Pose &parklot_pose, std::vector<int> &angle, std::vector<int> &velocity, std::vector<size_t> &run_time);
// void PoseInfoCallback(const nav_msgs::Odometry &msg);

// #endif