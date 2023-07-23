#pragma once

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
class imuPreintegration
{
private:
    ros::NodeHandle m_nh;
    ros::Publisher odom_pub_, low_freq_odom_pub_, path_pub_, gtsam_odom_pub_;
    ros::Subscriber imu_sub_, dlo_odom_sub_;
    double lastImuTime = -1;
    sensor_msgs::Imu lastImuData;
    //
    Eigen::Vector3d Pwb;              // position :    from  imu measurements
    Eigen::Quaterniond qwb = Eigen::Quaterniond(Eigen::Vector4d(0,0,0,1));            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw;          // velocity  :   from imu measurements
    Eigen::Vector3d gw = Eigen::Vector3d(0,0,-9.81);    // ENU frame
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