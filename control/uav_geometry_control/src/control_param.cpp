#include "control_param.h"

ControlParam::ControlParam(ros::NodeHandle &nh)
{
    nh.getParam("timeout/odom", odom_td);
    nh.getParam("timeout/cmd", cmd_td);
    
    takeoff_pose.pose.position.x = nh.param<double>("takeoff/init_x", 0.0);
    takeoff_pose.pose.position.y = nh.param<double>("takeoff/init_y", 0.0);
    takeoff_pose.pose.position.z = nh.param<double>("takeoff/init_z", 0.0);
    takeoff_pose.pose.orientation.w = 1;
    hover_pose.pose.position.x = nh.param<double>("takeoff/hover_x", 0.0);
    hover_pose.pose.position.y = nh.param<double>("takeoff/hover_y", 0.0);
    hover_pose.pose.position.z = nh.param<double>("takeoff/hover_z", 0.0);
    hover_pose.pose.orientation.w = 1;
    takeoff_delay = nh.param<double>("takeoff/delay", 0.0);
    
    double kpx = nh.param<double>("geometry_control/kpx", 0.0);
    double kpy = nh.param<double>("geometry_control/kpy", 0.0);
    double kpz = nh.param<double>("geometry_control/kpz", 0.0);
    double kvx = nh.param<double>("geometry_control/kvx", 0.0);
    double kvy = nh.param<double>("geometry_control/kvy", 0.0);
    double kvz = nh.param<double>("geometry_control/kvz", 0.0);
    double krx = nh.param<double>("geometry_control/krx", 0.0);
    double kry = nh.param<double>("geometry_control/kry", 0.0);
    double krz = nh.param<double>("geometry_control/krz", 0.0);
}
