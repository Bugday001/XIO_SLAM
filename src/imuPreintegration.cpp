/**
 * 将cloud变换到world坐标系，提供给fast-planner
 * 
 * 2023.01.14
*/
#include <imuPreintegration.h>

imuPreintegration::imuPreintegration()
    : m_nh("~")
{
    imu_sub_ = m_nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",
                                        2000,
                                        &imuPreintegration::imuHandler,
                                        this,
                                        ros::TransportHints().tcpNoDelay());
    dlo_odom_sub_ = m_nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom",
                                        2000,
                                        &imuPreintegration::dloHandler,
                                        this,
                                        ros::TransportHints().tcpNoDelay());
    odom_pub_ = m_nh.advertise<nav_msgs::Odometry>( "imu_incremental", 2000);
    gtsam_odom_pub_ = m_nh.advertise<nav_msgs::Odometry>( "gtsamOdom", 2000);
    path_pub_ = m_nh.advertise<nav_msgs::Path>("imu_path", 10, true);
    low_freq_odom_pub_ = m_nh.advertise<nav_msgs::Odometry>( "low_mavros", 2000);
    getLidar = 0;//state: 0没接受过，1更新了，-1还没更新
    qwb = Eigen::Quaterniond(Eigen::Vector4d(0,0,0,1));
    gw = Eigen::Vector3d(0,0,-9.81);
    //gtsam
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(10);
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(0.01, 2); // acc white noise in continuous
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(0.001, 2); // gyro white noise in continuous
    p->integrationCovariance =
        gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2); // error committed in integrating position from velocities
    gtsam::imuBias::ConstantBias
        prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());
    imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p,
                                                                prior_imu_bias);
    prevBias_ = gtsam::imuBias::ConstantBias();
    imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
}

void imuPreintegration::dloHandler(const nav_msgs::Odometry::ConstPtr &msg) {
    //降频
    static int cnt = 0;
    if(cnt<10) {
        cnt++;
        return;
    }
    cnt = 0;
    low_freq_odom_pub_.publish(*msg);
    geometry_msgs::Point point = msg->pose.pose.position;
    Pwb << point.x, point.y, point.z;
    Pwb << point.x, point.y, point.z;
    geometry_msgs::Quaternion ori = msg->pose.pose.orientation;
    qwb =  Eigen::Quaterniond(Eigen::Vector4d(ori.x, ori.y, ori.z, ori.w));;
    geometry_msgs::Vector3 v = msg->twist.twist.linear;
    Eigen::Vector3d tmpv(v.x, v.y, v.z);
    // Vw = (tmpv+Vw)/2;
    lastImuTime = msg->header.stamp.toSec();;
    getLidar = 1;
    //gtsam
    gtsam::Rot3 R = gtsam::Quaternion(ori.w, ori.x, ori.y, ori.z);
    prevStateOdom = gtsam::NavState(R, gtsam::Point3(point.x, point.y, point.z), 
                                                                                (gtsam::Vector(3) << Vw(0), Vw(1), Vw(2)).finished());
    imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
}

void imuPreintegration::imuHandler(const sensor_msgs::Imu::ConstPtr &imuData)
{
    double imuTime = imuData->header.stamp.toSec();;
    //判断是否第一帧IMU，是否有初始的lidar位置信息
    if(lastImuTime == -1 || getLidar == 0) {
        lastImuTime = imuTime;
        return;
    }
    double dt = imuTime - lastImuTime;
    lastImuTime = imuTime;
    lastImuData = *imuData;
    Eigen::Vector3d gyro ( lastImuData.angular_velocity.x,
                                                        lastImuData.angular_velocity.y, 
                                                        lastImuData.angular_velocity.z);
    Eigen::Vector3d acc (lastImuData.linear_acceleration.x,
                                                        lastImuData.linear_acceleration.y, 
                                                        lastImuData.linear_acceleration.z);
    //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
    Eigen::Quaterniond dq;
    Eigen::Vector3d dtheta_half =  gyro * dt /2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();
    dq.normalize();
    /// imu 动力学模型 欧拉积分
    Eigen::Vector3d acc_w = qwb * (acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw
    qwb = qwb * dq;
    Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
    Vw = Vw + acc_w * dt;

    //gtsam预积分
    imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(lastImuData.linear_acceleration.x,
                                                           lastImuData.linear_acceleration.y,
                                                           lastImuData.linear_acceleration.z),
                                            gtsam::Vector3(lastImuData.angular_velocity.x,
                                                           lastImuData.angular_velocity.y,
                                                           lastImuData.angular_velocity.z), dt);
    // predict odometry
    currentState = imuIntegratorImu_->predict(prevStateOdom, prevBias_);
    rosPublish(imuData->header);
    
}

void imuPreintegration::rosPublish(std_msgs::Header header) {
    //odom
    nav_msgs::Odometry odom;
    geometry_msgs::Point point;
    point.x = Pwb(0);
    point.y = Pwb(1);
    point.z = Pwb(2);
    geometry_msgs::Quaternion ori;
    ori.w = qwb.w();
    ori.x = qwb.x();
    ori.y = qwb.y();
    ori.z = qwb.z();
    geometry_msgs::Vector3 v;
    v.x = Vw(0);
    v.y = Vw(1);
    v.z = Vw(2);
    odom.header = header;
    odom.header.frame_id = "map";
    odom.twist.twist.linear = v;
    odom.pose.pose.position = point;
    odom.pose.pose.orientation = ori;
    odom_pub_.publish(odom);
    //path
    imuPath.header = header;
    imuPath.header.frame_id = "map";

    geometry_msgs::PoseStamped position_3d;
    position_3d.pose.position = point; 
    position_3d.pose.orientation = ori;
    position_3d.header = header;
    position_3d.header.frame_id = "map";
    imuPath.poses.push_back(position_3d);
    path_pub_.publish(imuPath);
    //gtsam pose
    gtsam::Pose3 lidarPose = gtsam::Pose3(currentState.quaternion(), currentState.position());

    odom.pose.pose.position.x = lidarPose.translation().x();
    odom.pose.pose.position.y = lidarPose.translation().y();
    odom.pose.pose.position.z = lidarPose.translation().z();
    odom.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
    odom.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
    odom.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
    odom.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

    odom.twist.twist.linear.x = currentState.velocity().x();
    odom.twist.twist.linear.y = currentState.velocity().y();
    odom.twist.twist.linear.z = currentState.velocity().z();
    // odom.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
    // odom.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
    // odom.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
    gtsam_odom_pub_.publish(odom);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "imuPreintegration");

    imuPreintegration SM;

    ros::spin();
    return 0;
}