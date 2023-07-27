#include <Eigen/Dense>
#include <Eigen/Core>

#include <vector>
#include <sensor_msgs/Imu.h>

using namespace Eigen;
namespace xio
{
    // imu预积分状态类
    class NavState
    {
    public:
        NavState(Vector3d p = Vector3d(0, 0, 0), Vector3d v = Vector3d(0, 0, 0), Quaterniond q = Quaterniond(Vector4d(0, 0, 0, 1)), Vector3d gw = Vector3d(0, 0, -9.81))
            : Pwb(p), qwb(q), Vw(v), gw(gw)
        {
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

        Eigen::Vector3d Pwb;    // position :    from  imu measurements
        Eigen::Quaterniond qwb; // quaterniond:  from imu measurements
        Eigen::Vector3d Vw;     // velocity  :   from imu measurements
        Eigen::Vector3d gw;     // ENU frame
    };

    class IMUPreintegration
    {
    public:
        // 预积分函数
        void imuPropagate(NavState & state, Eigen::Vector3d gyro, Eigen::Vector3d acc, double dt)
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
