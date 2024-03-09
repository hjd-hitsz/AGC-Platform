#pragma once
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>

class ControlParam
{
public:
    geometry_msgs::PoseStamped takeoff_pose;
    geometry_msgs::PoseStamped hover_pose;
    double takeoff_delay;
    
    double odom_td;
    double cmd_td;
    
    Eigen::Vector3d kp, kv, kr;
    double hover_thrust_rate;
    
    ControlParam(ros::NodeHandle &nh);
    ~ControlParam(){}
};