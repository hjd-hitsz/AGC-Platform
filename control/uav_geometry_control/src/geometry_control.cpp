#include "geometry_control.h"

Eigen::Vector3d GeometryControl::vee(Eigen::Matrix3d R)
{
    return Eigen::Vector3d(R(2, 1), R(0, 2), R(1, 0));
}

GeometryControl::GeometryControl(Eigen::Vector3d kp, Eigen::Vector3d kv, Eigen::Vector3d kr, double hover_thrust_rate)
    : kp_(kp), kv_(kv), kr_(kr), hover_thrust_rate_(hover_thrust_rate), g_(9.81)
{
}

Eigen::Vector4d GeometryControl::getControl(Eigen::Vector3d p, 
                                            Eigen::Vector3d v, 
                                            Eigen::Quaterniond q, 
                                            Eigen::Vector3d p_des, 
                                            Eigen::Vector3d v_des, 
                                            Eigen::Vector3d a_des,
                                            Eigen::Vector3d j_des, 
                                            double yaw_des, double yaw_rate_des)
{
    // define axis e3 and b3 from odometry
    auto R = q.toRotationMatrix();
    auto e3 = Eigen::Vector3d(0, 0, 1);
    auto b3 = R * e3;
    
    // compute thrust command and convert to thrust rate
    
    auto pe = p - p_des;
    auto ve = v - v_des;
    auto a_cmd = - kp_.cwiseProduct(pe) - kv_.cwiseProduct(ve) + g_ * e3 + a_des;
    auto a_norm_cmd = a_cmd.dot(b3);
    auto b3d = a_cmd.normalized();
    auto thrust_rate = std::min(hover_thrust_rate_ / g_ * a_norm_cmd, 1.0);
    // std::cout << "a_des" << a_des.transpose() << '\t'
    //           << "a_norm_cmd" << a_norm_cmd << '\t'
    //           << "pe" << pe.transpose() << std::endl;
    
    // compute desired attitude
    auto b1c = Eigen::Vector3d(cos(yaw_des), sin(yaw_des), 0);
    auto b2d = (b3d.cross(b1c)).normalized();
    auto b1d = b2d.cross(b3d);
    Eigen::Matrix3d R_des = Eigen::Matrix3d::Zero();
    R_des.col(0) = b1d;
    R_des.col(1) = b2d;
    R_des.col(2) = b3d;
    auto Re = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
    auto re = vee(Re);
    // std::cout << "re" << re.transpose() << std::endl;
    
    // compute desired angular velocity
    auto hw = (j_des - (b3d.dot(j_des) * b3d)) / a_norm_cmd;
    auto wx = - hw.dot(b2d);
    auto wy = hw.dot(b1d);
    auto wz = yaw_rate_des * e3.dot(b3d);
    Eigen::Vector3d w_des(wx, wy, wz);
    std::cout << "w_des" << w_des.transpose() << std::endl;
    
    // compute angular velocity command and output
    auto w_cmd = - kr_.cwiseProduct(re);// + R.transpose() * R_des * w_des
    Eigen::Vector4d control;
    control << thrust_rate, w_cmd(0), w_cmd(1), w_cmd(2);
    return control;
}
