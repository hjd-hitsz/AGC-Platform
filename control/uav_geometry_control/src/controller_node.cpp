#include <ros/ros.h>
#include "control_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nh("~");    
    
    ControlParam param(nh);
    ROS_INFO("[controller_node] param load");
    ControlInterface controller(param, nh);
    auto flatness_cmd_topic = nh.param<std::string>("flatness_cmd_topic", "/flatness_cmd");
    auto odom_topic = nh.param<std::string>("odom_topic", "/odom");
    auto state_sub = nh.subscribe("/mavros/state", 1, &ControlInterface::mavrosStateCallback, &controller);
    auto cmd_sub = nh.subscribe(flatness_cmd_topic, 1, &ControlInterface::cmdCallback, &controller);
    auto odom_sub = nh.subscribe(odom_topic, 1, &ControlInterface::odomCallback, &controller);
    controller.takeoff();
    ros::Rate rate(100);
    while (ros::ok()) {
        controller.commandControl();
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
