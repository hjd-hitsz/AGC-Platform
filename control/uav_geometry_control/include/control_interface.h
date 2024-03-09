#pragma once
#include "geometry_control.h"
#include "control_param.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <uav_geometry_control/flatness_polycoeffs.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>

class ControlInterface
{
private:
    ros::Time odom_stamp_;
    ros::Time cmd_stamp_;
    
    Eigen::Vector4d flatness_cmd_;
    Eigen::VectorXd odom_;
    mavros_msgs::State mavros_state_;
    
    ControlParam param_;
    
    std::shared_ptr<GeometryControl> gc_;
    Eigen::Matrix<double, 4, -1> coeffs_;
    double duration_, t0_;
    
    ros::Publisher local_pose_pub_;
    ros::Publisher attitude_pub_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient arm_client_;
public:
    ControlInterface(ControlParam &param, ros::NodeHandle &nh);
    ~ControlInterface(){}
    
    // callback
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg);
    void coeffsCallback(const uav_geometry_control::flatness_polycoeffs::ConstPtr& msg);
    
    // condition
    bool isTakeover(const Eigen::Vector3d &pose, const Eigen::Quaterniond &quat);
    
    // process
    void takeoff();
    void commandControl();
    Eigen::VectorXd polycommandResolve(double tf);
};