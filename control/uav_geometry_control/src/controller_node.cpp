#include <ros/ros.h>
#include "control_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("~");    
    
    ControlParam param(nh);
    ROS_INFO("[controller_node] param load");
    ControlInterface controller(param, nh);
    auto flatness_polycoeffs_topic = nh.param<std::string>("flatness_polycoeffs_topic", "/flatness_polycoeffs");
    auto odom_topic = nh.param<std::string>("odom_topic", "/odom");
    auto ctrl_rate = nh.param<double>("rate", 100);
    auto state_sub = nh.subscribe("/mavros/state", 1, &ControlInterface::mavrosStateCallback, &controller);
    auto odom_sub = nh.subscribe(odom_topic, 1, &ControlInterface::odomCallback, &controller);
    auto coeffs_sub = nh.subscribe(flatness_polycoeffs_topic, 1, &ControlInterface::coeffsCallback, &controller);
    controller.takeoff();
    ros::Rate rate(ctrl_rate);
    while (ros::ok()) {
        controller.commandControl();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
