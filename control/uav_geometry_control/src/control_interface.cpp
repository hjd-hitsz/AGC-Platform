#include "control_interface.h"

ControlInterface::ControlInterface(ControlParam &param, ros::NodeHandle &nh): param_(param)
{
    local_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    odom_stamp_ = ros::Time::now();
    cmd_stamp_ = ros::Time::now();
    odom_ = Eigen::VectorXd::Zero(10);
}

void ControlInterface::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom_ << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
             msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
             msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z;
    odom_stamp_ = msg->header.stamp;
}

void ControlInterface::cmdCallback(const uav_geometry_control::flatness_cmd::ConstPtr &msg)
{
    flatness_cmd_ << msg->x, msg->y, msg->z, msg->yaw;
    cmd_stamp_ = msg->header.stamp;
}

void ControlInterface::mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_state_ = *msg;
}

bool ControlInterface::isTakeover(const Eigen::Vector3d &pose, const Eigen::Quaterniond &quat)
{
    auto now = ros::Time::now();
    auto flag = true;
    
    if (now - odom_stamp_ > ros::Duration(param_.odom_td)) {
        ROS_WARN("[isTakeover-not] no odometry feedback");
        flag = false;
    }
    else {
        if (odom_.block<3, 1>(3, 0).norm() > 0.1) {
            ROS_WARN("[isTakeover-not] uav is not hovering");
            flag = false;
        }
    }
    if (now - cmd_stamp_ > ros::Duration(param_.cmd_td)) {
        ROS_WARN("[isTakeover-not] no command received");
        flag = false;
    }
    
    if (flag) ROS_INFO("[isTakeover-ok] take over by command");
    return flag;
}

void ControlInterface::takeoff()
{
    ros::Rate rate(50);
    // wait for FCU connection
    while(ros::ok() && !mavros_state_.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("[takeoff] Connected to FCU");

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pose_pub_.publish(param_.takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("[takeoff] Takeoff pose sent (100 times)");
    
    // switch to OFFBOARD
    ros::Time last_request = ros::Time::now();
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    Eigen::Vector3d cmd_pose(param_.hover_pose.pose.position.x, 
                             param_.hover_pose.pose.position.y, 
                             param_.hover_pose.pose.position.z);
    Eigen::Quaterniond cmd_quat(param_.hover_pose.pose.orientation.w,
                                param_.hover_pose.pose.orientation.x,
                                param_.hover_pose.pose.orientation.y,
                                param_.hover_pose.pose.orientation.z);
    auto pub_pose = param_.takeoff_pose;
    while (ros::ok())
    {
        if (mavros_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0))) {
            if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("[takeoff] Switched to OFFBOARD mode");
            }
            last_request = ros::Time::now();
            local_pose_pub_.publish(param_.takeoff_pose);
        }
        else if (!mavros_state_.armed && (ros::Time::now() - last_request > ros::Duration(2.0))) {
            if (arm_client_.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("[takeoff] Arming");
            }
            last_request = ros::Time::now();
            local_pose_pub_.publish(param_.takeoff_pose);
        }
        else {
            if (ros::Time::now() - last_request > ros::Duration(5.0)) {
                pub_pose = param_.hover_pose;
            }
            if (ros::Time::now() - last_request > ros::Duration(param_.takeoff_delay)) {
                
                if (isTakeover(cmd_pose, cmd_quat)) break;
                else last_request = ros::Time::now();
            }
            local_pose_pub_.publish(pub_pose);
            ros::spinOnce();
            rate.sleep();
        }
    }
    
}

void ControlInterface::commandControl()
{
    if (ros::Time::now() - cmd_stamp_ > ros::Duration(param_.cmd_td)) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = flatness_cmd_(0);
        pose.pose.position.y = flatness_cmd_(1);
        pose.pose.position.z = flatness_cmd_(2);
        double yaw = flatness_cmd_(3);
        pose.pose.orientation.w = cos(yaw / 2);
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = sin(yaw / 2);
        local_pose_pub_.publish(pose);
        ROS_WARN("[commandControl] no command received");
        return;
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = flatness_cmd_(0);
    pose.pose.position.y = flatness_cmd_(1);
    pose.pose.position.z = flatness_cmd_(2);
    double yaw = flatness_cmd_(3);
    pose.pose.orientation.w = cos(yaw / 2);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin(yaw / 2);
    local_pose_pub_.publish(pose);
    return;
}
