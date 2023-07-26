#include<Eigen/Dense>
#include<Eigen/Core>

#include<vector>
using namespace Eigen;
namespace xio {
    //imu预积分状态类
    class NavState
    {
    public:
        NavState(Vector3d p=Vector3d(0,0,0), Vector3d v=Vector3d(0,0,0), Quaterniond q=Quaterniond(Vector4d(0, 0, 0, 1)), Vector3d gw=Vector3d(0,0,9.81))
        : Pwb(p), qwb(q), Vw(v), gw(gw){

        }
        /**
         * 更新状态
        */
        void updateState(Vector3d p=Vector3d(0,0,0), Vector3d v=Vector3d(0,0,0), Quaterniond q=Quaterniond(Vector4d(0, 0, 0, 1)))
        {
            Pwb = p;
            qwb = q;
            Vw = v;
        }

        Eigen::Vector3d Pwb;              // position :    from  imu measurements
        Eigen::Quaterniond qwb;        // quaterniond:  from imu measurements
        Eigen::Vector3d Vw;          // velocity  :   from imu measurements
        Eigen::Vector3d gw;    // ENU frame
    };
    //预积分函数
    void imuPropagate(NavState& state, Eigen::Vector3d gyro, Eigen::Vector3d acc, double dt);
}


