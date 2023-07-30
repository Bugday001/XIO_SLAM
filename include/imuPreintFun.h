#ifndef IMU_PREINTFUN_
#define IMU_PREINTFUN_

#include <vector>
#include <sensor_msgs/Imu.h>
#include "mathFun.h"

using namespace Eigen;
namespace xio
{
    // imu预积分状态类
    class NavState
    {
    public:
        NavState(Vector3d p = Vector3d(0, 0, 0), Vector3d v = Vector3d(0, 0, 0), 
        Quaterniond q = Quaterniond(Vector4d(0, 0, 0, 1)),
        Eigen::Vector3d bg = Vector3d(0, 0, 0), Eigen::Vector3d ba = Vector3d(0, 0, 0))
            : Pwb(p), qwb(q), Vw(v), bg_(bg), ba_(ba){
            }
        /**
         * 更新状态
         */
        void updateState(Vector3d p = Vector3d(0, 0, 0), Vector3d v = Vector3d(0, 0, 0), Quaterniond q = Quaterniond(Vector4d(0, 0, 0, 1)))
        {
            Pwb = p;
            qwb = q;
            Vw = v;
        }

        void print() {
            std::cout<<"State:"<<std::endl;
            std::cout<<"point:"<<std::endl<<Pwb<<std::endl;
            std::cout<<"q:"<<std::endl<<qwb.matrix()<<std::endl;
            std::cout<<"v:"<<std::endl<<Vw<<std::endl;
            std::cout<<"bg_:"<<std::endl<<bg_<<std::endl;
            std::cout<<"ba_:"<<std::endl<<ba_<<std::endl;
        }

        Eigen::Vector3d Pwb;    // position :    from  imu measurements
        Eigen::Quaterniond qwb; // quaterniond:  from imu measurements
        Eigen::Vector3d Vw;     // velocity  :   from imu measurements
        Eigen::Vector3d bg_;  // gyro 零偏
        Eigen::Vector3d ba_;  // acce 零偏
    };

    class IMUPreintegration
    {
    public:
        Eigen::Vector3d dv_, dp_;
        Eigen::Quaterniond dq_;
        double dt_;
        Eigen::Vector3d bg_, ba_;
        Eigen::Vector3d gw_;  // ENU frame
        // 雅可比矩阵
        Matrix3d dR_dbg_ = Matrix3d::Zero();
        Matrix3d dV_dbg_ = Matrix3d::Zero();
        Matrix3d dV_dba_ = Matrix3d::Zero();
        Matrix3d dP_dbg_ = Matrix3d::Zero();
        Matrix3d dP_dba_ = Matrix3d::Zero();
        Eigen::Matrix<double, 9, 9> cov_ = Eigen::Matrix<double, 9, 9>::Zero();              // 累计噪声矩阵
        Eigen::Matrix<double, 6, 6> noise_gyro_acce_ = Eigen::Matrix<double, 6, 6>::Zero();  // 测量噪声矩阵
    public:
        IMUPreintegration() {
            dv_ << 0,0,0;
            dp_ << 0,0,0;
            bg_ <<0,0,0;
            ba_ <<0,0,0;
            dq_ = Quaterniond(Vector4d(0, 0, 0, 1));
            gw_ = Eigen::Vector3d(0, 0,-9.81);
            dt_= 0;
            double noise_gyro_ = 1e-2, noise_acce_ = 1e-1;
            const double ng2 = noise_gyro_ * noise_gyro_;
            const double na2 = noise_acce_ * noise_acce_;
            noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
        }
        /**
         * reset param
        */
        void resetIntegrationAndSetBias(Eigen::Vector3d bg, Eigen::Vector3d ba) {
            dv_ << 0,0,0;
            dp_ << 0,0,0;
            dq_ = Quaterniond(Vector4d(0, 0, 0, 1));
            bg_  = bg;
            ba_ = ba;
            dt_ = 0;
        }

        void print() {
            std::cout<<"preint State:"<<std::endl;
            std::cout<<"dp_:"<<std::endl<<dp_<<std::endl;
            std::cout<<"dq_:"<<std::endl<<dq_.matrix()<<std::endl;
            std::cout<<"dv_:"<<std::endl<<dv_<<std::endl;
            std::cout<<"bg_:"<<std::endl<<bg_<<std::endl;
            std::cout<<"ba_:"<<std::endl<<ba_<<std::endl;
            std::cout<<"cov:"<<std::endl<<cov_<<std::endl;
        }

        // 预积分函数
        void imuPropagate(Eigen::Vector3d gyro, Eigen::Vector3d acc, double dt)
        {
            gyro = gyro - bg_;
            acc = acc - ba_;
            // delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
            Eigen::Quaterniond dq;
            Eigen::Vector3d dtheta_half = gyro * dt / 2.0;
            dq.w() = 1;
            dq.x() = dtheta_half.x();
            dq.y() = dtheta_half.y();
            dq.z() = dtheta_half.z();
            dq.normalize();
            /// imu 动力学模型 欧拉积分
            Eigen::Vector3d acc_w = dq_ * (acc); 
            dp_ = dp_ + dv_ * dt + 0.5 * dt * dt * acc_w;
            dv_ = dv_ + acc_w * dt;

            // 运动方程雅可比矩阵系数，A,B阵
            // 另外两项在后面
            Eigen::Matrix<double, 9, 9> A;
            A.setIdentity();
            Eigen::Matrix<double, 9, 6> B;
            B.setZero();

            Matrix3d acc_hat = Sophus::SO3d::hat(acc);
            // Eigen::Matrix3d acc_hat;
            // acc_hat << 0, -acc(2), acc(1),
            //                         acc(2), 0, -acc(0),
            //                         -acc(1), acc(0), 0;
            double dt2 = dt * dt;

            // NOTE A, B左上角块与公式稍有不同
            A.block<3, 3>(3, 0) = -dq_.matrix() * dt * acc_hat;
            A.block<3, 3>(6, 0) = -0.5f * dq_.matrix() * acc_hat * dt2;
            A.block<3, 3>(6, 3) = dt * Matrix3d::Identity();

            B.block<3, 3>(3, 3) = dq_.matrix() * dt;
            B.block<3, 3>(6, 3) = 0.5f * dq_.matrix() * dt2;

            // 更新各雅可比，见式(4.39)
            dP_dba_ = dP_dba_ + dV_dba_ * dt - 0.5f * dq_.matrix() * dt2;                      // (4.39d)
            dP_dbg_ = dP_dbg_ + dV_dbg_ * dt - 0.5f * dq_.matrix() * dt2 * acc_hat * dR_dbg_;  // (4.39e)
            dV_dba_ = dV_dba_ - dq_.matrix() * dt;                                             // (4.39b)
            dV_dbg_ = dV_dbg_ - dq_.matrix() * dt * acc_hat * dR_dbg_;                         // (4.39c)


            // 旋转部分
            Vector3d omega = gyro * dt;         // 转动量
            Matrix3d rightJ = xio::jr(omega);  // 右雅可比
            // std::cout<<"omega"<<omega<<std::endl;
            Sophus::SO3d deltaR = Sophus::SO3d::exp(omega);   // exp后
            dq_ = dq_ * dq;

            A.block<3, 3>(0, 0) = deltaR.matrix().transpose();
            B.block<3, 3>(0, 0) = rightJ * dt;

            // 更新噪声项
            cov_ = A * cov_ * A.transpose() + B * noise_gyro_acce_ * B.transpose();

            // 更新dR_dbg
            dR_dbg_ = deltaR.matrix().transpose() * dR_dbg_ - rightJ * dt;  // (4.39a)
            //叠加积分的时间
            dt_ += dt;
        }

        Sophus::SO3d GetDeltaRotation(const Vector3d &bg) { 
            Sophus::SO3d SO3_q(dq_);
            // std::cout<<"GetDeltaRotation"<<std::endl<<dR_dbg_<<std::endl<<bg<<std::endl<<bg_<<std::endl;
            return  SO3_q * Sophus::SO3d::exp(dR_dbg_ * (bg - bg_)); 
        }

        Eigen::Vector3d GetDeltaVelocity(const Vector3d &bg, const Vector3d &ba) {
            return dv_ + dV_dbg_ * (bg - bg_) + dV_dba_ * (ba - ba_);
        }

        Eigen::Vector3d GetDeltaPosition(const Vector3d &bg, const Vector3d &ba) {
            return dp_ + dP_dbg_ * (bg - bg_) + dP_dba_ * (ba - ba_);
        }

        NavState Predict(const NavState &start) {
            Eigen::Quaterniond Rj = start.qwb * dq_;
            Eigen::Vector3d vj = start.qwb * dv_ + start.Vw + gw_ * dt_;
            Eigen::Vector3d pj = start.qwb * dp_ + start.Pwb + start.Vw * dt_ + 0.5f * gw_ * dt_ * dt_;

            auto state = NavState(pj, vj, Rj);
            state.bg_ = bg_;
            state.ba_ = ba_;
            return state;
        }
    };
    

    class ParamServer
    {
    public:
        Eigen::Matrix3d extRot;
        Eigen::Quaterniond extQRPY;
        int imuType;
        ParamServer()
        {
            // imu旋转
            extRot << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
            Eigen::Matrix3d extRPY;
            extRPY << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
            extQRPY = Eigen::Quaterniond(extRPY).inverse();
            imuType = 1;
        }
        sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in)
        {
            sensor_msgs::Imu imu_out = imu_in;
            // rotate acceleration
            Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
            acc = extRot * acc;
            imu_out.linear_acceleration.x = acc.x();
            imu_out.linear_acceleration.y = acc.y();
            imu_out.linear_acceleration.z = acc.z();
            // rotate gyroscope
            Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
            gyr = extRot * gyr;
            imu_out.angular_velocity.x = gyr.x();
            imu_out.angular_velocity.y = gyr.y();
            imu_out.angular_velocity.z = gyr.z();
            // rotate roll pitch yaw
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y,
                                      imu_in.orientation.z);
            Eigen::Quaterniond q_final;
            if (imuType == 0)
            {
                q_final = extQRPY;
            }
            else if (imuType == 1)
                q_final = q_from * extQRPY;
            else
                std::cout << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;

            q_final.normalize();
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();

            // if (sqrt(
            //         q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() +
            //         q_final.w() * q_final.w()) < 0.1)
            // {
            //     ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            //     ros::shutdown();
            // }

            return imu_out;
        }
    };
}

#endif