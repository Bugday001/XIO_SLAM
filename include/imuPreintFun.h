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
                time_stamp_ = 0;
            }
        /**
         * 更新状态
         */
        void updateState(Vector3d p = Vector3d(0, 0, 0), Vector3d v = Vector3d(0, 0, 0), 
                                                Quaterniond q = Quaterniond(Vector4d(0, 0, 0, 1)), double time_stamp = -1)
        {
            Pwb = p;
            qwb = q;
            Vw = v;
            time_stamp_ = time_stamp;
        }

        void print() {
            std::cout<<"State:"<<std::endl;
            std::cout<<"point:"<<std::endl<<Pwb<<std::endl;
            std::cout<<"q:"<<std::endl<<qwb.matrix()<<std::endl;
            std::cout<<"v:"<<std::endl<<Vw<<std::endl;
            std::cout<<"bg_:"<<std::endl<<bg_<<std::endl;
            std::cout<<"ba_:"<<std::endl<<ba_<<std::endl;
            std::cout<<"time stamp:"<<time_stamp_<<std::endl;
        }

        /**
         * 返回Sophus::SE3d
        */
        Sophus::SE3d GetSE3() const {
            return Sophus::SE3d(qwb.matrix(), Pwb);
        }
        Eigen::Vector3d Pwb;    // position :    from  imu measurements
        Eigen::Quaterniond qwb; // quaterniond:  from imu measurements
        Eigen::Vector3d Vw;     // velocity  :   from imu measurements
        Eigen::Vector3d bg_;  // gyro 零偏
        Eigen::Vector3d ba_;  // acce 零偏
        double time_stamp_;
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
        Eigen::Matrix<double, 9, 9> cov_;              // 累计噪声矩阵
        Eigen::Matrix<double, 6, 6> noise_gyro_acce_;  // 测量噪声矩阵
    public:
        IMUPreintegration() {
            dv_ << 0,0,0;
            dp_ << 0,0,0;
            bg_ <<0,0,0;
            ba_ <<0,0,0;
            dq_ = Quaterniond(Vector4d(0, 0, 0, 1));
            gw_ = Eigen::Vector3d(0, 0,-9.81);
            dt_= 0;
            cov_ = Eigen::Matrix<double, 9, 9>::Zero();
            dR_dbg_ = Matrix3d::Zero();
            dV_dbg_ = Matrix3d::Zero();
            dV_dba_ = Matrix3d::Zero();
            dP_dbg_ = Matrix3d::Zero();
            dP_dba_ = Matrix3d::Zero();
            noise_gyro_acce_ = Eigen::Matrix<double, 6, 6>::Zero();
            double noise_gyro_ = 1e-2, noise_acce_ = 1e-1;
            const double ng2 = noise_gyro_ * noise_gyro_;
            const double na2 = noise_acce_ * noise_acce_;
            noise_gyro_acce_.diagonal() << ng2, ng2, ng2, na2, na2, na2;
        }
        IMUPreintegration(IMUPreintegration& imupreint) {
            dv_ = imupreint.dv_;
            dp_ = imupreint.dp_;
            bg_ = imupreint.bg_;
            ba_ = imupreint.ba_;
            dq_ = imupreint.dq_;   
            gw_ = Eigen::Vector3d(0, 0,-9.81);
            dt_ = imupreint.dt_;
            cov_ = imupreint.cov_;
            dR_dbg_ = imupreint.dR_dbg_;
            dV_dbg_ = imupreint.dV_dbg_;
            dV_dba_ = imupreint.dV_dba_;
            dP_dbg_ = imupreint.dP_dbg_;
            dP_dba_ = imupreint.dP_dba_;
            noise_gyro_acce_ = Eigen::Matrix<double, 6, 6>::Zero();
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
            dt_ = 0;
            cov_ = Eigen::Matrix<double, 9, 9>::Zero();
            dR_dbg_ = Matrix3d::Zero();
            dV_dbg_ = Matrix3d::Zero();
            dV_dba_ = Matrix3d::Zero();
            dP_dbg_ = Matrix3d::Zero();
            dP_dba_ = Matrix3d::Zero();

            bg_  = bg;
            ba_ = ba;
        }

        void print() {
            std::cout<<"preint State:"<<std::endl;
            std::cout<<"dt_: "<<dt_<<std::endl;
            std::cout<<"dp_:"<<std::endl<<dp_<<std::endl;
            std::cout<<"dq_:"<<std::endl<<dq_.matrix()<<std::endl;
            std::cout<<"dv_:"<<std::endl<<dv_<<std::endl;
            std::cout<<"bg_:"<<std::endl<<bg_<<std::endl;
            std::cout<<"ba_:"<<std::endl<<ba_<<std::endl;
            std::cout<<"cov:"<<std::endl<<std::fixed<<std::setprecision(6)<<cov_<<std::endl;
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
    /**
     * pose 插值算法
     * @tparam T    数据类型
     * @param query_time 查找时间
     * @param data  数据
     * @param take_pose_func 从数据中取pose的谓词
     * @param result 查询结果
     * @param best_match_iter 查找到的最近匹配
     *
     * NOTE 要求query_time必须在data最大时间和最小时间之间，不会外推
     * data的map按时间排序
     * @return 插值是否成功
     */
    template <typename T>
    bool PoseInterp(double query_time, const std::vector<std::pair<T, double>>& data, const std::function<Sophus::SE3d(const T&)>& take_pose_func,
                    Sophus::SE3d& result) {
        if (data.empty()) {
            std::cout << "data is empty";
            return false;
        }

        if (query_time > data.rbegin()->second) {
            // std::cout << "query time is later than last, " << std::setprecision(18) << ", query: " << query_time
            //         << ", end time: " << data.rbegin()->second;
            return false;
        }
        
        auto match_iter = data.begin();
        for (auto iter = data.begin(); iter != data.end(); ++iter) {
            auto next_iter = iter;
            next_iter++;

            if (iter->second < query_time && next_iter->second >= query_time) {
                match_iter = iter;
                break;
            }
        }

        auto match_iter_n = match_iter;
        match_iter_n++;
        assert(match_iter_n != data.end());

        double dt = match_iter_n->second - match_iter->second;
        double s = (query_time - match_iter->second) / dt;  // s=0 时为第一帧，s=1时为next

        Sophus::SE3d pose_first = take_pose_func(match_iter->first);
        Sophus::SE3d pose_next = take_pose_func(match_iter_n->first);
        result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
                pose_first.translation() * (1 - s) + pose_next.translation() * s};
        // best_match = s < 0.5 ? match_iter->first : match_iter_n->first;
        return true;
    }
}
namespace xio {
    enum class SensorType { INIT, OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN };
}
#endif