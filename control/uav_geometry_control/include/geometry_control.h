#pragma once
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

class GeometryControl
{
private:
    Eigen::Vector3d kp_;
    Eigen::Vector3d kv_;
    Eigen::Vector3d kr_;
    double hover_thrust_rate_;
    double g_;
    /// @brief vee map from SO(3) to R3
    /// @param R SO(3) matrix
    /// @return R3 vector
    Eigen::Vector3d vee(Eigen::Matrix3d R);
public:
    /// @brief Construct a new Geometry Control object
    /// @param kp gain of position error
    /// @param kv gain of velocity error
    /// @param kr gain of angular velocity error
    /// @param kr gain of SO(3) attitude error
    /// @param hover_thrust_rate thrust_rate at the hover position in 0 and 1
    GeometryControl(Eigen::Vector3d kp, 
                    Eigen::Vector3d kv, 
                    Eigen::Vector3d kr, 
                    double hover_thrust_rate);
    ~GeometryControl(){}
    /// @brief compute geometry control command (remember to transform all translation inputs to local/map frame first)
    /// @details reference paper1: Geometric tracking control of a quadrotor UAV on SE(3)
    /// reference paper2: Minimum snap trajectory generation and control for quadrotors
    /// @param p position from odometry or other feedbacks
    /// @param v velocity from odometry or other feedbacks
    /// @param q attitude from odometry or other feedbacks
    /// @param p_des desired position
    /// @param v_des desired velocity
    /// @param a_des desired acceleration
    /// @param j_des desired jerk needed to compute desired angular velocity
    /// @param yaw_des desired yaw
    /// @param yaw_rate_des desired yaw rate
    /// @return control command: thrust_rate, wx, wy, wz
    Eigen::Vector4d getControl(Eigen::Vector3d p,
                               Eigen::Vector3d v,
                               Eigen::Quaterniond q,
                               Eigen::Vector3d p_des,
                               Eigen::Vector3d v_des,
                               Eigen::Vector3d a_des,
                               Eigen::Vector3d j_des,
                               double yaw_des,
                               double yaw_rate_des);
};