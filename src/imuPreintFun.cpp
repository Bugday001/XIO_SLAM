/**
 * imu预积分函数
 *
 * 2023.07.26
 */
#include "imuPreintFun.h"

void xio::imuPropagate(NavState& state, Eigen::Vector3d gyro, Eigen::Vector3d acc, double dt)
{
    // delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
    Eigen::Quaterniond dq;
    Eigen::Vector3d dtheta_half = gyro * dt / 2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    dq.normalize();
    /// imu 动力学模型 欧拉积分
    Eigen::Vector3d acc_w = state.qwb * (acc) + state.gw; // aw = Rwb * ( acc_body - acc_bias ) + gw
    state.qwb = state.qwb * dq;
    state.Pwb = state.Pwb + state.Vw * dt + 0.5 * dt * dt * acc_w;
    state.Vw = state.Vw + acc_w * dt;
}