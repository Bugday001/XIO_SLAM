#ifndef IMU_PREINTERATION
#define IMU_PREINTERATION

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include<Eigen/Dense>
#include<Eigen/Core>

#include<vector>

#include "imuPreintFun.h"
class imuPreintegration
{
private:
    std::mutex mtx;
    
    ros::NodeHandle m_nh;
    ros::Publisher odom_pub_, low_freq_odom_pub_, path_pub_, gtsam_odom_pub_;
    ros::Subscriber imu_sub_, dlo_odom_sub_;
    double lastImuTime = -1;
    sensor_msgs::Imu lastImuData;
    std::deque<sensor_msgs::Imu> imuQueImu;
    const double delta_t = 0;
    //lidar
    double lastImuQT = -1;
    double currentCorrectionTime;  //最新的lidar odom 时间
    //eigin preintegration
    xio::NavState xioState;
    int getLidar;
    nav_msgs::Path imuPath;
    //gtsam
    gtsam::imuBias::ConstantBias prevBias_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;
    gtsam::NavState currentState, prevStateOdom;
    //imu rotation
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;
    int imuType = 0;
public:
    imuPreintegration();

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in);

    void dloHandler(const nav_msgs::Odometry::ConstPtr &msg);

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuData);

    void rosPublish(std_msgs::Header header);
};

#endif