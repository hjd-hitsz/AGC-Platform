#include "control_interface.h"

ControlInterface::ControlInterface(ControlParam &param, ros::NodeHandle &nh): param_(param)
{
    local_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    attitude_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arm_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    
    gc_.reset(new GeometryControl(param.kp, param.kv, param.kr, param.hover_thrust_rate));
    
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

void ControlInterface::mavrosStateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_state_ = *msg;
}

void ControlInterface::coeffsCallback(const uav_geometry_control::flatness_polycoeffs::ConstPtr &msg)
{
    int dim = msg->dim;
    coeffs_.resize(Eigen::NoChange_t{}, dim + 1);
    for (int i = 0; i < dim + 1; ++i) {
        coeffs_(0, i) = msg->x_coeff[i].data;
        coeffs_(1, i) = msg->y_coeff[i].data;
        coeffs_(2, i) = msg->z_coeff[i].data;
        coeffs_(3, i) = msg->yaw_coeff[i].data;
    }
    duration_ = msg->tf;
    t0_ = msg->header.stamp.toSec();
    cmd_stamp_ = ros::Time::now();
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
    double tf = ros::Time::now().toSec() - t0_;
    tf = std::min(tf, duration_);
    auto cmd = polycommandResolve(tf);
    // std::cout << "tf: " << tf << '\t' << "cmd: " << cmd.transpose() << std::endl;
    Eigen::Vector3d p_des, v_des, a_des, j_des, p, v;
    p_des << cmd(0), cmd(1), cmd(2);
    v_des << cmd(3), cmd(4), cmd(5);
    a_des << cmd(6), cmd(7), cmd(8);
    j_des << cmd(9), cmd(10), cmd(11);
    p << odom_(0), odom_(1), odom_(2);
    v << odom_(3), odom_(4), odom_(5);
    Eigen::Quaterniond q(odom_(6), odom_(7), odom_(8), odom_(9));
    double yaw_des = cmd(12), yaw_dot_des = cmd(13);
    auto gc_cmd = gc_->getControl(p, v, q, p_des, v_des, a_des, j_des, yaw_des, yaw_dot_des);
    mavros_msgs::AttitudeTarget msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
    msg.thrust = gc_cmd(0);
    msg.body_rate.x = gc_cmd(1);
    msg.body_rate.y = gc_cmd(2);
    msg.body_rate.z = gc_cmd(3);
    attitude_pub_.publish(msg);
}

Eigen::VectorXd ControlInterface::polycommandResolve(double tf)
{
    Eigen::VectorXd cmd = Eigen::VectorXd::Zero(14);
    Eigen::Matrix<double, 6, 1> t0_span, t1_span, t2_span, t3_span;
    t0_span << 1, tf, tf*tf, tf*tf*tf, tf*tf*tf*tf, tf*tf*tf*tf*tf;
    t1_span << 0, 1, 2*tf, 3*tf*tf, 4*tf*tf*tf, 5*tf*tf*tf*tf;
    t2_span << 0, 0, 2, 6*tf, 12*tf*tf, 20*tf*tf*tf;
    t3_span << 0, 0, 0, 6, 24*tf, 60*tf*tf;
    auto x = coeffs_.block<1, 6>(0, 0).transpose().dot(t0_span);
    auto y = coeffs_.block<1, 6>(1, 0).transpose().dot(t0_span);
    auto z = coeffs_.block<1, 6>(2, 0).transpose().dot(t0_span);
    auto yaw = coeffs_.block<1, 6>(3, 0).transpose().dot(t0_span);
    auto vx = coeffs_.block<1, 6>(0, 0).transpose().dot(t1_span);
    auto vy = coeffs_.block<1, 6>(1, 0).transpose().dot(t1_span);
    auto vz = coeffs_.block<1, 6>(2, 0).transpose().dot(t1_span);
    auto yaw_dot = coeffs_.block<1, 6>(3, 0).transpose().dot(t1_span);
    auto ax = coeffs_.block<1, 6>(0, 0).transpose().dot(t2_span);
    auto ay = coeffs_.block<1, 6>(1, 0).transpose().dot(t2_span);
    auto az = coeffs_.block<1, 6>(2, 0).transpose().dot(t2_span);
    auto jx = coeffs_.block<1, 6>(0, 0).transpose().dot(t3_span);
    auto jy = coeffs_.block<1, 6>(1, 0).transpose().dot(t3_span);
    auto jz = coeffs_.block<1, 6>(2, 0).transpose().dot(t3_span);
    cmd << x, y, z, vx, vy, vz, ax, ay, az, jx, jy, jz, yaw, yaw_dot;
    return cmd;
}
