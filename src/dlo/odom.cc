/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/odom.h"

std::atomic<bool> dlo::OdomNode::abort_(false);


/**
 * Constructor
 **/

dlo::OdomNode::OdomNode(ros::NodeHandle node_handle) : nh(node_handle) {

  this->getParams();

  this->stop_publish_thread = false;
  this->stop_publish_keyframe_thread = false;
  this->stop_metrics_thread = false;
  this->stop_debug_thread = false;

  this->dlo_initialized = false;
  this->imu_calibrated = false;  //bugday 数据集不含标定时间，跳过该步骤

  this->icp_sub = this->nh.subscribe("pointcloud", 1, &dlo::OdomNode::icpCB, this);
  this->imu_sub = this->nh.subscribe("imu", 1, &dlo::OdomNode::imuCB, this);
  this->gt_sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth/state", 100, &dlo::OdomNode::gtCB, this); 
  
  this->odom_pub = this->nh.advertise<nav_msgs::Odometry>("odom", 1);
  this->pose_pub = this->nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  this->kf_pub = this->nh.advertise<nav_msgs::Odometry>("kfs", 1, true);
  this->keyframe_pub = this->nh.advertise<sensor_msgs::PointCloud2>("keyframe", 1, true);
  this->cur_cloud_t_pub = this->nh.advertise<sensor_msgs::PointCloud2>("cur_cloud_t", 1, true);
  this->imu_odom_pub_ = this->nh.advertise<nav_msgs::Odometry>("dio_imu_odom", 1, true);

  this->publish_timer = this->nh.createTimer(ros::Duration(0.01), &dlo::OdomNode::publishPose, this);

  this->odom.pose.pose.position.x = 0.;
  this->odom.pose.pose.position.y = 0.;
  this->odom.pose.pose.position.z = 0.;
  this->odom.pose.pose.orientation.w = 1.;
  this->odom.pose.pose.orientation.x = 0.;
  this->odom.pose.pose.orientation.y = 0.;
  this->odom.pose.pose.orientation.z = 0.;
  this->odom.pose.covariance = {0.};

  this->origin = Eigen::Vector3f(0., 0., 0.);

  this->T = Eigen::Matrix4f::Identity();
  this->T_s2s = Eigen::Matrix4f::Identity();

  this->pose_s2s = Eigen::Vector3f(0., 0., 0.);
  this->rotq_s2s = Eigen::Quaternionf(1., 0., 0., 0.);

  this->pose = Eigen::Vector3f(0., 0., 0.);
  this->rotq = Eigen::Quaternionf(1., 0., 0., 0.);

  this->imu_SE3 = Eigen::Matrix4f::Identity();

  this->imu_bias.gyro.x = 0.;
  this->imu_bias.gyro.y = 0.;
  this->imu_bias.gyro.z = 0.;
  this->imu_bias.accel.x = 0.;
  this->imu_bias.accel.y = 0.;
  this->imu_bias.accel.z = 0.;

  this->imu_meas.stamp = 0.;
  this->imu_meas.ang_vel.x = 0.;
  this->imu_meas.ang_vel.y = 0.;
  this->imu_meas.ang_vel.z = 0.;
  this->imu_meas.lin_accel.x = 0.;
  this->imu_meas.lin_accel.y = 0.;
  this->imu_meas.lin_accel.z = 0.;

  this->imu_buffer.set_capacity(this->imu_buffer_size_);
  this->first_imu_time = 0.;

  this->original_scan = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->current_scan = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->current_scan_t = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);

  this->keyframe_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->num_keyframes = 0;

  this->submap_cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->submap_hasChanged = true;
  this->submap_kf_idx_prev.clear();

  this->source_cloud = nullptr;
  this->target_cloud = nullptr;

  this->convex_hull.setDimension(3);
  this->concave_hull.setDimension(3);
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);
  this->concave_hull.setKeepInformation(true);

  this->gicp.setCorrespondenceRandomness(this->gicps2m_k_correspondences_);
  this->gicp.setMaxCorrespondenceDistance(this->gicps2m_max_corr_dist_);
  this->gicp.setMaximumIterations(this->gicps2m_max_iter_);
  this->gicp.setTransformationEpsilon(this->gicps2m_transformation_ep_);
  this->gicp.setEuclideanFitnessEpsilon(this->gicps2m_euclidean_fitness_ep_);
  this->gicp.setRANSACIterations(this->gicps2m_ransac_iter_);
  this->gicp.setRANSACOutlierRejectionThreshold(this->gicps2m_ransac_inlier_thresh_);

  pcl::Registration<PointType, PointType>::KdTreeReciprocalPtr temp;
  this->gicp.setSearchMethodSource(temp, true);
  this->gicp.setSearchMethodTarget(temp, true);

  this->crop.setNegative(true);
  this->crop.setMin(Eigen::Vector4f(-this->crop_size_, -this->crop_size_, -this->crop_size_, 1.0));
  this->crop.setMax(Eigen::Vector4f(this->crop_size_, this->crop_size_, this->crop_size_, 1.0));

  this->vf_scan.setLeafSize(this->vf_scan_res_, this->vf_scan_res_, this->vf_scan_res_);
  this->vf_submap.setLeafSize(this->vf_submap_res_, this->vf_submap_res_, this->vf_submap_res_);

  this->metrics.spaciousness.push_back(0.);

  // CPU Specs
  char CPUBrandString[0x40];
  memset(CPUBrandString, 0, sizeof(CPUBrandString));
  this->cpu_type = "";

#ifdef HAS_CPUID
  unsigned int CPUInfo[4] = {0,0,0,0};
  __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
  unsigned int nExIds = CPUInfo[0];
  for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
    __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
    if (i == 0x80000002)
      memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000003)
      memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
    else if (i == 0x80000004)
      memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
  }

  this->cpu_type = CPUBrandString;
  boost::trim(this->cpu_type);
#endif

  FILE* file;
  struct tms timeSample;
  char line[128];

  this->lastCPU = times(&timeSample);
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;

  file = fopen("/proc/cpuinfo", "r");
  this->numProcessors = 0;
  while(fgets(line, 128, file) != NULL) {
      if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
  }
  fclose(file);

  lastImuT_imu = -1;
  prev_point_ << 0,0,0;
  imuPreinteg_ = std::make_shared<xio::IMUPreintegration> ();
  imuPreinteg_Opt_ = std::make_shared<xio::IMUPreintegration> ();
  imuType = 1;
  lastImuT_opt = -1;

  ROS_INFO("DLO Odom Node Initialized");
}


/**
 * Destructor
 **/

dlo::OdomNode::~OdomNode() {}



/**
 * Odom Node Parameters
 **/

void dlo::OdomNode::getParams() {

  // Version
  ros::param::param<std::string>("~dlo/version", this->version_, "0.0.0");

  // Frames
  ros::param::param<std::string>("~dlo/odomNode/odom_frame", this->odom_frame, "odom");
  ros::param::param<std::string>("~dlo/odomNode/child_frame", this->child_frame, "base_link");

  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;
  this->child_frame = ns + "/" + this->child_frame;

  // Gravity alignment
  ros::param::param<bool>("~dlo/gravityAlign", this->gravity_align_, false);

  // Keyframe Threshold
  ros::param::param<double>("~dlo/odomNode/keyframe/threshD", this->keyframe_thresh_dist_, 0.1);
  ros::param::param<double>("~dlo/odomNode/keyframe/threshR", this->keyframe_thresh_rot_, 1.0);

  // Submap
  ros::param::param<int>("~dlo/odomNode/submap/keyframe/knn", this->submap_knn_, 10);
  ros::param::param<int>("~dlo/odomNode/submap/keyframe/kcv", this->submap_kcv_, 10);
  ros::param::param<int>("~dlo/odomNode/submap/keyframe/kcc", this->submap_kcc_, 10);

  // Initial Position
  ros::param::param<bool>("~dlo/odomNode/initialPose/use", this->initial_pose_use_, false);

  double px, py, pz, qx, qy, qz, qw;
  ros::param::param<double>("~dlo/odomNode/initialPose/position/x", px, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/position/y", py, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/position/z", pz, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/w", qw, 1.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/x", qx, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/y", qy, 0.0);
  ros::param::param<double>("~dlo/odomNode/initialPose/orientation/z", qz, 0.0);
  this->initial_position_ = Eigen::Vector3f(px, py, pz);
  this->initial_orientation_ = Eigen::Quaternionf(qw, qx, qy, qz);

  // Crop Box Filter
  ros::param::param<bool>("~dlo/odomNode/preprocessing/cropBoxFilter/use", this->crop_use_, false);
  ros::param::param<double>("~dlo/odomNode/preprocessing/cropBoxFilter/size", this->crop_size_, 1.0);

  // Voxel Grid Filter
  ros::param::param<bool>("~dlo/odomNode/preprocessing/voxelFilter/scan/use", this->vf_scan_use_, true);
  ros::param::param<double>("~dlo/odomNode/preprocessing/voxelFilter/scan/res", this->vf_scan_res_, 0.05);
  ros::param::param<bool>("~dlo/odomNode/preprocessing/voxelFilter/submap/use", this->vf_submap_use_, false);
  ros::param::param<double>("~dlo/odomNode/preprocessing/voxelFilter/submap/res", this->vf_submap_res_, 0.1);

  // Adaptive Parameters
  ros::param::param<bool>("~dlo/adaptiveParams", this->adaptive_params_use_, false);

  // IMU
  ros::param::param<bool>("~dlo/imu", this->imu_use_, false);
  ros::param::param<int>("~dlo/odomNode/imu/calibTime", this->imu_calib_time_, 3);
  ros::param::param<int>("~dlo/odomNode/imu/bufferSize", this->imu_buffer_size_, 2000);

  // GICP
  ros::param::param<int>("~dlo/odomNode/gicp/minNumPoints", this->gicp_min_num_points_, 100);
  ros::param::param<int>("~dlo/odomNode/gicp/s2m/kCorrespondences", this->gicps2m_k_correspondences_, 20);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/maxCorrespondenceDistance", this->gicps2m_max_corr_dist_, std::sqrt(std::numeric_limits<double>::max()));
  ros::param::param<int>("~dlo/odomNode/gicp/s2m/maxIterations", this->gicps2m_max_iter_, 64);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/transformationEpsilon", this->gicps2m_transformation_ep_, 0.0005);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/euclideanFitnessEpsilon", this->gicps2m_euclidean_fitness_ep_, -std::numeric_limits<double>::max());
  ros::param::param<int>("~dlo/odomNode/gicp/s2m/ransac/iterations", this->gicps2m_ransac_iter_, 0);
  ros::param::param<double>("~dlo/odomNode/gicp/s2m/ransac/outlierRejectionThresh", this->gicps2m_ransac_inlier_thresh_, 0.05);

  ros::param::param<bool>("~dlo/odomNode/debugVerbose",   this->debugVerbose, false);
  ros::param::param<std::vector<double>>("~dlo/odomNode/imu/extrinsicRot", extRotV, std::vector<double>());
  ros::param::param<std::vector<double>>("~dlo/odomNode/imu/extrinsicRPY", extRPYV, std::vector<double>());
  ros::param::param<std::vector<double>>("~dlo/odomNode/imu/extrinsicTrans", extTransV, std::vector<double>());
  Eigen::Matrix3d extRPY;
  extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
  extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
  // extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
  extQRPY = Eigen::Quaterniond(extRPY).inverse();
  this->deskew_status = false;
  this->first_valid_scan = false;

  ros::param::param<bool>("~dlo/deskew", this->deskew_, true);

}


/**
 * Start Odom Thread
 **/

void dlo::OdomNode::start() {
  ROS_INFO("Starting DLO Odometry Node");

  printf("\033[2J\033[1;1H");
  std::cout << std::endl << "==== Direct LiDAR Odometry v" << this->version_ << " ====" << std::endl << std::endl;

}


/**
 * Stop Odom Thread
 **/

void dlo::OdomNode::stop() {
  ROS_WARN("Stopping DLO Odometry Node");

  this->stop_publish_thread = true;
  if (this->publish_thread.joinable()) {
    this->publish_thread.join();
  }

  this->stop_publish_keyframe_thread = true;
  if (this->publish_keyframe_thread.joinable()) {
    this->publish_keyframe_thread.join();
  }

  this->stop_metrics_thread = true;
  if (this->metrics_thread.joinable()) {
    this->metrics_thread.join();
  }

  this->stop_debug_thread = true;
  if (this->debug_thread.joinable()) {
    this->debug_thread.join();
  }

  ros::shutdown();
}


/**
 * Abort Timer Callback
 **/

void dlo::OdomNode::abortTimerCB(const ros::TimerEvent& e) {
  if (abort_) {
    stop();
  }
}


/**
 * Publish to ROS
 **/

void dlo::OdomNode::publishToROS() {
  this->publishTransform();
  this->publishCloud();
}

/**
 * 发布高频imu odom
*/
void dlo::OdomNode::publishImuOdom() {
  //publish 临时
  nav_msgs::Odometry odom;
  geometry_msgs::Point point;
  point.x = xioState.Pwb(0);
  point.y = xioState.Pwb(1);
  point.z = xioState.Pwb(2);
  geometry_msgs::Quaternion ori;
  ori.w = xioState.qwb.w();
  ori.x = xioState.qwb.x();
  ori.y = xioState.qwb.y();
  ori.z = xioState.qwb.z();
  geometry_msgs::Vector3 v;
  v.x =xioState.Vw(0);
  v.y = xioState.Vw(1);
  v.z = xioState.Vw(2);
  odom.header = imuQueImu.back().header;
  odom.header.frame_id = this->odom_frame;
  odom.twist.twist.linear = v;
  odom.pose.pose.position = point;
  odom.pose.pose.orientation = ori;
  imu_odom_pub_.publish(odom);
}

void dlo::OdomNode::publishCloud() {
  sensor_msgs::PointCloud2 cur_cloud_t_ros;
  pcl::toROSMsg(*this->current_scan, cur_cloud_t_ros);
  cur_cloud_t_ros.header.stamp = this->scan_stamp;
  cur_cloud_t_ros.header.frame_id = this->odom_frame;
  this->cur_cloud_t_pub.publish(cur_cloud_t_ros);
}

/**
 * Publish Pose
 **/

void dlo::OdomNode::publishPose(const ros::TimerEvent& e) {

  this->odom.pose.pose.position.x = this->xioState.Pwb[0];
  this->odom.pose.pose.position.y = this->xioState.Pwb[1];
  this->odom.pose.pose.position.z = this->xioState.Pwb[2];

  this->odom.pose.pose.orientation.w = this->xioState.qwb.w();
  this->odom.pose.pose.orientation.x = this->xioState.qwb.x();
  this->odom.pose.pose.orientation.y = this->xioState.qwb.y();
  this->odom.pose.pose.orientation.z = this->xioState.qwb.z();

  this->odom.header.stamp = this->imu_stamp;
  this->odom.header.frame_id = this->odom_frame;
  this->odom.child_frame_id = this->child_frame;
  this->odom_pub.publish(this->odom);

  this->pose_ros.header.stamp = this->imu_stamp;
  this->pose_ros.header.frame_id = this->odom_frame;

  this->pose_ros.pose.position.x = this->xioState.Pwb[0];
  this->pose_ros.pose.position.y = this->xioState.Pwb[1];
  this->pose_ros.pose.position.z = this->xioState.Pwb[2];

  this->pose_ros.pose.orientation.w = this->xioState.qwb.w();
  this->pose_ros.pose.orientation.x = this->xioState.qwb.x();
  this->pose_ros.pose.orientation.y = this->xioState.qwb.y();
  this->pose_ros.pose.orientation.z = this->xioState.qwb.z();

  this->pose_pub.publish(this->pose_ros);
}


/**
 * Publish Transform
 **/

void dlo::OdomNode::publishTransform() {

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = this->scan_stamp;
  transformStamped.header.frame_id = this->odom_frame;
  transformStamped.child_frame_id = this->child_frame;

  transformStamped.transform.translation.x = this->xioState.Pwb[0];
  transformStamped.transform.translation.y = this->xioState.Pwb[1];
  transformStamped.transform.translation.z = this->xioState.Pwb[2];

  transformStamped.transform.rotation.w = this->xioState.qwb.w();
  transformStamped.transform.rotation.x = this->xioState.qwb.x();
  transformStamped.transform.rotation.y = this->xioState.qwb.y();
  transformStamped.transform.rotation.z = this->xioState.qwb.z();

  br.sendTransform(transformStamped);

}


/**
 * Publish Keyframe Pose and Scan
 **/

void dlo::OdomNode::publishKeyframe() {

  // Publish keyframe pose
  this->kf.header.stamp = this->scan_stamp;
  this->kf.header.frame_id = this->odom_frame;
  this->kf.child_frame_id = this->child_frame;

  this->kf.pose.pose.position.x = this->lidarState.Pwb[0];
  this->kf.pose.pose.position.y = this->lidarState.Pwb[1];
  this->kf.pose.pose.position.z = this->lidarState.Pwb[2];

  this->kf.pose.pose.orientation.w = this->lidarState.qwb.w();
  this->kf.pose.pose.orientation.x = this->lidarState.qwb.x();
  this->kf.pose.pose.orientation.y = this->lidarState.qwb.y();
  this->kf.pose.pose.orientation.z = this->lidarState.qwb.z();

  this->kf_pub.publish(this->kf);

  // Publish keyframe scan
  if (this->keyframe_cloud->points.size() == this->keyframe_cloud->width * this->keyframe_cloud->height) {
    sensor_msgs::PointCloud2 keyframe_cloud_ros;
    pcl::toROSMsg(*this->keyframe_cloud, keyframe_cloud_ros);
    keyframe_cloud_ros.header.stamp = this->scan_stamp;
    keyframe_cloud_ros.header.frame_id = this->odom_frame;
    this->keyframe_pub.publish(keyframe_cloud_ros);
  }

}


/**
 * Preprocessing
 **/

void dlo::OdomNode::preprocessPoints(const sensor_msgs::PointCloud2ConstPtr& pc) {

    // If there are too few points in the pointcloud, try again
  this->current_scan = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*pc, *this->current_scan);
  // automatically detect sensor type
  this->sensor = xio::SensorType::UNKNOWN;
  for (auto &field : pc->fields) {
    if (field.name == "t") {
      this->sensor = xio::SensorType::OUSTER;
      break;
    } else if (field.name == "time") {
      this->sensor = xio::SensorType::VELODYNE;
      break;
    } else if (field.name == "timestamp") {
      this->sensor = xio::SensorType::HESAI;
      break;
    }
  }

  if (this->sensor == xio::SensorType::UNKNOWN) {
    this->deskew_ = false;
  }
  *this->original_scan = *this->current_scan;

  if(this->deskew_ ) {
    this->deskewPointcloud();
  }
  else {
    imuOptPreint(this->curr_frame_stamp);
  }

  // Remove NaNs
  std::vector<int> idx;
  this->current_scan->is_dense = false;
  pcl::removeNaNFromPointCloud(*this->current_scan, *this->current_scan, idx);

  // Crop Box Filter
  if (this->crop_use_) {
    this->crop.setInputCloud(this->current_scan);
    this->crop.filter(*this->current_scan);
  }
  // Voxel Grid Filter
  if (this->vf_scan_use_) {
    pcl::PointCloud<PointType>::Ptr current_scan_
    (boost::make_shared<pcl::PointCloud<PointType>>(*this->current_scan));
    this->vf_scan.setInputCloud(current_scan_);
    this->vf_scan.filter(*current_scan_);
    this->current_scan = current_scan_;
  }
  // else {
  //   this->current_scan = this->deskewed_scan;
  // }

}

void dlo::OdomNode::deskewPointcloud() {
  double sweep_ref_time = this->curr_frame_stamp;
  //获取时间
  std::function<double(PointType)> extract_point_time;
  if (this->sensor == xio::SensorType::OUSTER) {
    extract_point_time = [&sweep_ref_time](PointType pt)
      { return sweep_ref_time + pt.t * 1e-9f; };

  } else if (this->sensor == xio::SensorType::VELODYNE) {
    extract_point_time = [&sweep_ref_time](PointType pt)
      { return sweep_ref_time - pt.time; };

  } else if (this->sensor == xio::SensorType::HESAI) {
    extract_point_time = [&sweep_ref_time](PointType pt)
      { return pt.timestamp; };
  }
  this->totalPointNums = this->current_scan->size();
  auto point = (*this->current_scan)[this->totalPointNums-1];
  double end_time = extract_point_time(point);

  //两帧之间imu积分
  imuOptPreint(end_time);
  if(imuOptInitialized == false || imu_states_.empty()) {
    return;
  }
  this->deskewNums = 0;
  xio::NavState last_imu = imu_states_.back().first;
  Sophus::SE3d T_end(last_imu.qwb, last_imu.Pwb);
  for (auto& point: *this->current_scan) {
      Sophus::SE3d Ti = T_end;
      bool success = xio::PoseInterp<xio::NavState>(
          extract_point_time(point), imu_states_,
          [](const xio::NavState &s) { return s.GetSE3(); }, Ti);
      if(success) this->deskewNums++;
      Eigen::Vector3d pi = Eigen::Vector3d(point.x, point.y, point.z);
      Eigen::Vector3d p_compensate = T_end.inverse() * Ti * pi;
      point.x = p_compensate(0);
      point.y = p_compensate(1);
      point.z = p_compensate(2);
  }
}


/**
 * Initialize Input Target
 **/

void dlo::OdomNode::initializeInputTarget() {

  this->prev_frame_stamp = this->curr_frame_stamp;

  // initialize keyframes
  pcl::PointCloud<PointType>::Ptr first_keyframe (new pcl::PointCloud<PointType>);
  pcl::transformPointCloud (*this->current_scan, *first_keyframe, this->T);

  // voxelization for submap
  if (this->vf_submap_use_) {
    this->vf_submap.setInputCloud(first_keyframe);
    this->vf_submap.filter(*first_keyframe);
  }

  // keep history of keyframes
  this->keyframes.push_back(std::make_pair(std::make_pair(this->pose, this->rotq), first_keyframe));
  *this->keyframe_cloud = *first_keyframe;

  this->keyframe_normals.push_back(this->gicp.getSourceCovariances());
  this->publish_keyframe_thread = std::thread( &dlo::OdomNode::publishKeyframe, this );
  this->publish_keyframe_thread.detach();

  ++this->num_keyframes;

}


/**
 * Set Input Sources
 **/

void dlo::OdomNode::setInputSources(){
  this->gicp.setInputSource(this->current_scan);
  this->gicp.calculateSourceCovariances();
}


/**
 * Gravity Alignment
 **/

void dlo::OdomNode::gravityAlign() {

  // get average acceleration vector for 1 second and normalize
  Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
  const double then = ros::Time::now().toSec();
  int n=0;
  while ((ros::Time::now().toSec() - then) < 1.) {
    lin_accel[0] += this->imu_meas.lin_accel.x;
    lin_accel[1] += this->imu_meas.lin_accel.y;
    lin_accel[2] += this->imu_meas.lin_accel.z;
    ++n;
  }
  lin_accel[0] /= n; lin_accel[1] /= n; lin_accel[2] /= n;

  // normalize
  double lin_norm = sqrt(pow(lin_accel[0], 2) + pow(lin_accel[1], 2) + pow(lin_accel[2], 2));
  lin_accel[0] /= lin_norm; lin_accel[1] /= lin_norm; lin_accel[2] /= lin_norm;

  // define gravity vector (assume point downwards)
  Eigen::Vector3f grav;
  grav << 0, 0, 1;

  // calculate angle between the two vectors
  Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(lin_accel, grav);

  // normalize
  double grav_norm = sqrt(grav_q.w()*grav_q.w() + grav_q.x()*grav_q.x() + grav_q.y()*grav_q.y() + grav_q.z()*grav_q.z());
  grav_q.w() /= grav_norm; grav_q.x() /= grav_norm; grav_q.y() /= grav_norm; grav_q.z() /= grav_norm;

  // set gravity aligned orientation
  this->rotq = grav_q;
  this->T.block(0,0,3,3) = this->rotq.toRotationMatrix();
  this->T_s2s.block(0,0,3,3) = this->rotq.toRotationMatrix();

  // rpy
  auto euler = grav_q.toRotationMatrix().eulerAngles(2, 1, 0);
  double yaw = euler[0] * (180.0/M_PI);
  double pitch = euler[1] * (180.0/M_PI);
  double roll = euler[2] * (180.0/M_PI);

  std::cout << "done" << std::endl;
  std::cout << "  Roll [deg]: " << roll << std::endl;
  std::cout << "  Pitch [deg]: " << pitch << std::endl << std::endl;
}


/**
 * Initialize 6DOF
 **/

void dlo::OdomNode::initializeDLO() {

  // Calibrate IMU
  if (!this->imu_calibrated && this->imu_use_) {
    return;
  }

  // Gravity Align
  if (this->gravity_align_ && this->imu_use_ && this->imu_calibrated && !this->initial_pose_use_) {
    std::cout << "Aligning to gravity... "; std::cout.flush();
    this->gravityAlign();
  }

  // Use initial known pose
  if (this->initial_pose_use_) {
    std::cout << "Setting known initial pose... "; std::cout.flush();

    // set known position
    this->pose = this->initial_position_;
    this->T.block(0,3,3,1) = this->pose;
    this->T_s2s.block(0,3,3,1) = this->pose;
    this->origin = this->initial_position_;

    // set known orientation
    this->rotq = this->initial_orientation_;
    this->T.block(0,0,3,3) = this->rotq.toRotationMatrix();
    this->T_s2s.block(0,0,3,3) = this->rotq.toRotationMatrix();

    std::cout << "done" << std::endl << std::endl;
  }

  this->dlo_initialized = true;
  std::cout << "DLO initialized! Starting localization..." << std::endl;

}


sensor_msgs::Imu dlo::OdomNode::imuConverter(const sensor_msgs::Imu &imu_in)
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

/**
 * ICP Point Cloud Callback
 **/
void dlo::OdomNode::icpCB(const sensor_msgs::PointCloud2ConstPtr& pc) {

  double then = ros::Time::now().toSec();
  this->scan_stamp = pc->header.stamp;
  this->curr_frame_stamp = pc->header.stamp.toSec();

  // DLO Initialization procedures (IMU calib, gravity align)
  if (!this->dlo_initialized) {
    this->initializeDLO();
    return;
  }

  // Preprocess points Deskews
  this->preprocessPoints(pc);
  if (this->current_scan->points.size() < this->gicp_min_num_points_) {
    ROS_WARN("Low number of points!");
    return;
  }
  // Compute Metrics
  this->metrics_thread = std::thread( &dlo::OdomNode::computeMetrics, this );
  this->metrics_thread.detach();

  // Set Adaptive Parameters
  if (this->adaptive_params_use_){
    this->setAdaptiveParams();
  }

  // Set new frame as input source for both gicp objects
  this->setInputSources();

  // Set initial frame as target
  if (this->keyframes.size() == 0) {
    this->initializeInputTarget();
    return;
  }
  
  this->comp_times.push_back(ros::Time::now().toSec() - then);
  // Get the next pose via IMU + S2M
  this->getNextPose();
  // Update current keyframe poses and map
  this->updateKeyframes();
  // Update trajectory
  this->trajectory.push_back( std::make_pair(this->pose, this->rotq) );

  // Update some statistics
  this->comp_times.push_back(ros::Time::now().toSec() - then);

  // imu 积分
  if(imuOptInitialized) {
    imuPreintegration();
  }
  this->comp_times.push_back(ros::Time::now().toSec() - then);
  //Update next point
  this->prev_point_ = this->pose.cast<double>();
  // Update next time stamp
  this->prev_frame_stamp = this->curr_frame_stamp;
  //Update lidarState
  this->preLidarState = this->lidarState;
  // Publish stuff to ROS
  this->publish_thread = std::thread( &dlo::OdomNode::publishToROS, this );
  this->publish_thread.detach();

  // Debug statements and publish custom DLO message
  this->debug_thread = std::thread( &dlo::OdomNode::debug, this );
  this->debug_thread.detach();
}

/**
 * 对两帧之间的imu进行积分
 * return 是否初始化了
*/
void dlo::OdomNode::imuOptPreint(double end_time) 
{
    //为优化的imu进行积分
  // 0. initialize system
  if (imuOptInitialized == false)
  {
    // pop old IMU message
    while (!imuQueOpt.empty())
    {
        if (imuQueOpt.front().header.stamp.toSec() < end_time - delta_t)
        {
            lastImuT_opt = imuQueOpt.front().header.stamp.toSec();
            imuQueOpt.pop_front();
        }
        else
            break;
    }
    imuOptInitialized = true;
    return;
    // return false;
  }
  if (this->imuQueOpt.empty() || imuQueOpt.front().header.stamp.toSec() < end_time) {
    // Wait for the latest IMU data
    //使用 std::unique_lock(通过 std::mutex) 来锁住当前线程
    std::unique_lock<decltype(this->mtx_imu)> lock(this->mtx_imu);
    this->cv_imu_stamp.wait(lock, [this, &end_time]{ return imuQueOpt.back().header.stamp.toSec() >= end_time; });
  }
  imu_states_.clear();
  // 1. integrate imu data and optimize
  while (!imuQueOpt.empty())
  {
      // pop and integrate imu data that is between two optimizations
      sensor_msgs::Imu *thisImu = &imuQueOpt.front();
      double imuTime = imuQueOpt.front().header.stamp.toSec();
      if (imuTime < end_time - delta_t)
      {
          double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
          Eigen::Vector3d gyro(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z);
          Eigen::Vector3d acc(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z);
          imuPreinteg_Opt_->imuPropagate(gyro, acc, dt);
          imu_states_.push_back({imuPreinteg_Opt_->Predict(this->lidarState), imuTime});
          lastImuT_opt = imuTime;
          imuQueOpt.pop_front();
      }
      else
          break;
  }
  // return true;
}


/**
 * 接收到lidar的odom时进行预积分
*/
void dlo::OdomNode::imuPreintegration() {
  double lastImuQT = -1;
  if(imuQueImu.empty())
    return;
  
  double dt_odom = this->curr_frame_stamp - this->prev_frame_stamp;
  Eigen::Vector3d Vw = (this->pose.cast<double>()-this->prev_point_)/dt_odom;  //使用两个lidar位置计算速度作为积分初值
  this->lidarState.updateState(this->pose.cast<double>(), Vw, this->rotq.cast<double>(), this->curr_frame_stamp);

  
  //优化
  Optimize();
  imuPreinteg_Opt_->resetIntegrationAndSetBias(this->lidarState.bg_, this->lidarState.ba_);


  while (!imuQueImu.empty() && imuQueImu.front().header.stamp.toSec() < this->curr_frame_stamp - delta_t) {
      lastImuQT = imuQueImu.front().header.stamp.toSec();
      imuQueImu.pop_front();
  }
  // repropogate
  if (!imuQueImu.empty()) {
    // reset bias use the newly optimized bias
    // TO DO
    imuPreinteg_->resetIntegrationAndSetBias(this->lidarState.bg_, this->lidarState.ba_);
    // std::cout<<"bg"<<this->lidarState.bg_<<"ba:" <<this->lidarState.ba_<<std::endl;
    // integrate imu message from the beginning of this optimization
    for (int i = 0; i < (int)imuQueImu.size(); ++i) {
        sensor_msgs::Imu *thisImu = &imuQueImu[i];
        double imuTime = thisImu->header.stamp.toSec();
        double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);
        Eigen::Vector3d gyro(thisImu->angular_velocity.x, thisImu->angular_velocity.y, thisImu->angular_velocity.z);
        Eigen::Vector3d acc(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z);
        imuPreinteg_->imuPropagate(gyro, acc, dt);
        lastImuQT = imuTime;
    }
  }
}


/**
 * ground truth Callback
*/
void dlo::OdomNode::gtCB(const nav_msgs::OdometryConstPtr &PointRes)
{
	this->uwbPoint = PointRes->pose.pose.position;
}


/**
 * IMU Callback
 **/

void dlo::OdomNode::imuCB(const sensor_msgs::Imu::ConstPtr& imu) {

  if (!this->imu_use_) {
    return;
  }

  sensor_msgs::Imu thisImu = imuConverter(*imu);
  this->imu_stamp = thisImu.header.stamp;

  double ang_vel[3], lin_accel[3];

  // Get IMU samples
  ang_vel[0] = thisImu.angular_velocity.x;
  ang_vel[1] = thisImu.angular_velocity.y;
  ang_vel[2] = thisImu.angular_velocity.z;

  lin_accel[0] = thisImu.linear_acceleration.x;
  lin_accel[1] = thisImu.linear_acceleration.y;
  lin_accel[2] = thisImu.linear_acceleration.z;

  if (this->first_imu_time == 0.) {
    this->first_imu_time = thisImu.header.stamp.toSec();
  }

  // IMU calibration procedure - do for three seconds
  if (!this->imu_calibrated) {

    static int num_samples = 0;
    static bool print = true;

    if ((thisImu.header.stamp.toSec() - this->first_imu_time) < this->imu_calib_time_) {

      num_samples++;

      this->imu_bias.gyro.x += ang_vel[0];
      this->imu_bias.gyro.y += ang_vel[1];
      this->imu_bias.gyro.z += ang_vel[2];

      this->imu_bias.accel.x += lin_accel[0];
      this->imu_bias.accel.y += lin_accel[1];
      this->imu_bias.accel.z += lin_accel[2];

      if(print) {
        std::cout << "Calibrating IMU for " << this->imu_calib_time_ << " seconds... "; std::cout.flush();
        print = false;
      }

    } else {

      this->imu_bias.gyro.x /= num_samples;
      this->imu_bias.gyro.y /= num_samples;
      this->imu_bias.gyro.z /= num_samples;

      this->imu_bias.accel.x /= num_samples;
      this->imu_bias.accel.y /= num_samples;
      this->imu_bias.accel.z /= num_samples;
      Eigen::Vector3d bg(this->imu_bias.gyro.x, this->imu_bias.gyro.y, this->imu_bias.gyro.z);
      Eigen::Vector3d ba(this->imu_bias.accel.x, this->imu_bias.accel.y, this->imu_bias.accel.z);
      imuPreinteg_->resetIntegrationAndSetBias(bg, ba);
      imuPreinteg_Opt_->resetIntegrationAndSetBias(bg, ba);
      this->imu_calibrated = true;

      std::cout << "done" << std::endl;
      std::cout << "  Gyro biases [xyz]: " << this->imu_bias.gyro.x << ", " << this->imu_bias.gyro.y << ", " << this->imu_bias.gyro.z << std::endl << std::endl;

    }

  } else {
    // Store into circular buffer
    this->mtx_imu.lock();
    this->imu_buffer.push_front(this->imu_meas);
    this->imuQueOpt.push_back(thisImu);
    this->imuQueImu.push_back(thisImu);
    double imuTime = thisImu.header.stamp.toSec();
    double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
    Eigen::Vector3d gyro(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z);
    Eigen::Vector3d acc(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z);
    imuPreinteg_->imuPropagate(gyro, acc, dt);
    xioState = imuPreinteg_->Predict(lidarState);
    xioState.time_stamp_ = imuTime;
    lastImuT_imu = imuTime;
    publishImuOdom();
    this->mtx_imu.unlock();
    this->cv_imu_stamp.notify_one();
  }

}

/**
 * 紧耦合，进行优化
*/
void dlo::OdomNode::Optimize() {
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  auto *solver = new g2o::OptimizationAlgorithmLevenberg(
      g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  // std::cout<<"last:"<<std::endl;
  // preLidarState.print();
  // std::cout<<"current:"<<std::endl;
  // lidarState.print();
  // 上时刻顶点， pose, v, bg, ba
  auto v0_pose = new VertexPose();
  v0_pose->setId(0);
  v0_pose->setEstimate(Sophus::SE3d(this->preLidarState.qwb,this->preLidarState.Pwb));
  optimizer.addVertex(v0_pose);

  auto v0_vel = new VertexVelocity();
  v0_vel->setId(1);
  v0_vel->setEstimate(this->preLidarState.Vw);
  optimizer.addVertex(v0_vel);

  auto v0_bg = new VertexGyroBias();
  v0_bg->setId(2);
  v0_bg->setEstimate(this->preLidarState.bg_);
  optimizer.addVertex(v0_bg);

  auto v0_ba = new VertexAccBias();
  v0_ba->setId(3);
  v0_ba->setEstimate(this->preLidarState.ba_);
  optimizer.addVertex(v0_ba);

  // 本时刻顶点，pose, v, bg, ba
  auto v1_pose = new VertexPose();
  v1_pose->setId(4);
  v1_pose->setEstimate(Sophus::SE3d(this->lidarState.qwb,this->lidarState.Pwb));  // gicp pose作为初值
  optimizer.addVertex(v1_pose);

  auto v1_vel = new VertexVelocity();
  v1_vel->setId(5);
  v1_vel->setEstimate(this->lidarState.Vw);
  optimizer.addVertex(v1_vel);

  auto v1_bg = new VertexGyroBias();
  v1_bg->setId(6);
  v1_bg->setEstimate(this->lidarState.bg_);
  optimizer.addVertex(v1_bg);

  auto v1_ba = new VertexAccBias();
  v1_ba->setId(7);
  v1_ba->setEstimate(this->lidarState.ba_);
  optimizer.addVertex(v1_ba);

   // imu factor
  if(this->debugVerbose) {
    this->imuPreinteg_Opt_->print();
  }
  // std::cout<<imuPreinteg_Opt_->dt_<<",  "<<lidarState.time_stamp_-preLidarState.time_stamp_<<std::endl;
  auto edge_inertial = new EdgeInertial(imuPreinteg_Opt_);
  edge_inertial->setVertex(0, v0_pose);
  edge_inertial->setVertex(1, v0_vel);
  edge_inertial->setVertex(2, v0_bg);
  edge_inertial->setVertex(3, v0_ba);
  edge_inertial->setVertex(4, v1_pose);
  edge_inertial->setVertex(5, v1_vel);
  auto *rk = new g2o::RobustKernelHuber();
  rk->setDelta(200.0);
  edge_inertial->setRobustKernel(rk);
  optimizer.addEdge(edge_inertial);

  // 零偏随机游走
  auto *edge_gyro_rw = new EdgeGyroRW();
  edge_gyro_rw->setVertex(0, v0_bg);
  edge_gyro_rw->setVertex(1, v1_bg);
  edge_gyro_rw->setInformation(Eigen::Matrix3d::Identity()*1e4);
  optimizer.addEdge(edge_gyro_rw);

  auto *edge_acc_rw = new EdgeAccRW();
  edge_acc_rw->setVertex(0, v0_ba);
  edge_acc_rw->setVertex(1, v1_ba);
  edge_acc_rw->setInformation(Eigen::Matrix3d::Identity()*1e4);
  optimizer.addEdge(edge_acc_rw);

  // 上一帧pose, vel, bg, ba的先验
  auto *edge_prior = new EdgePriorPoseNavState(this->preLidarState, prior_info_);
  edge_prior->setVertex(0, v0_pose);
  edge_prior->setVertex(1, v0_vel);
  edge_prior->setVertex(2, v0_bg);
  edge_prior->setVertex(3, v0_ba);
  optimizer.addEdge(edge_prior);

  /// 使用gicp的pose进行观测
  auto *edge_ndt = new EdgeGNSS(v1_pose, Sophus::SE3d(this->lidarState.qwb,this->lidarState.Pwb));
  Eigen::Matrix<double, 6, 6> inform = Eigen::Matrix<double, 6, 6>::Identity(); 
  double ndt_pos_noise_ = 0.1, ndt_ang_noise_ = 2.0 * 3.1415/180;
  double gp2 = ndt_pos_noise_ * ndt_pos_noise_;
  double ga2 = ndt_ang_noise_ * ndt_ang_noise_;
  inform.diagonal() << 1.0 / ga2, 1.0 / ga2, 1.0 / ga2, 1.0 / gp2, 1.0 / gp2, 1.0 / gp2;
  edge_ndt->setInformation(inform);
  optimizer.addEdge(edge_ndt);

  v0_bg->setFixed(true);
  v0_ba->setFixed(true);

  // go
  optimizer.setVerbose(false);
  optimizer.initializeOptimization();
  optimizer.optimize(20);

  // get results
  this->lidarState.qwb = v1_pose->estimate().so3().unit_quaternion(); ;
  this->lidarState.Pwb = v1_pose->estimate().translation();
  this->lidarState.Vw = v1_vel->estimate();
  this->lidarState.bg_ = v1_bg->estimate();
  this->lidarState.ba_ = v1_ba->estimate();
  if(this->debugVerbose) {
    this->lidarState.print();
  }
  // 计算当前时刻先验
  // 构建hessian
  // 15x2，顺序：v0_pose, v0_vel, v0_bg, v0_ba, v1_pose, v1_vel, v1_bg, v1_ba
  //            0       6        9     12     15        21      24     27
  Eigen::Matrix<double, 30, 30> H;
  H.setZero();

  H.block<24, 24>(0, 0) += edge_inertial->GetHessian();

  Eigen::Matrix<double, 6, 6> Hgr = edge_gyro_rw->GetHessian();
  H.block<3, 3>(9, 9) += Hgr.block<3, 3>(0, 0);
  H.block<3, 3>(9, 24) += Hgr.block<3, 3>(0, 3);
  H.block<3, 3>(24, 9) += Hgr.block<3, 3>(3, 0);
  H.block<3, 3>(24, 24) += Hgr.block<3, 3>(3, 3);

  Eigen::Matrix<double, 6, 6> Har = edge_acc_rw->GetHessian();
  H.block<3, 3>(12, 12) += Har.block<3, 3>(0, 0);
  H.block<3, 3>(12, 27) += Har.block<3, 3>(0, 3);
  H.block<3, 3>(27, 12) += Har.block<3, 3>(3, 0);
  H.block<3, 3>(27, 27) += Har.block<3, 3>(3, 3);

  H.block<15, 15>(0, 0) += edge_prior->GetHessian();
  H.block<6, 6>(15, 15) += edge_ndt->GetHessian();

  H = xio::Marginalize(H, 0, 14);
  prior_info_ = H.block<15, 15>(15, 15);

  if (false) {
      LOG(INFO) << "info trace: " << prior_info_.trace();
      LOG(INFO) << "optimization done.";
  }

  // NormalizeVelocity();
  // last_nav_state_ = current_nav_state_;
}

/**
 * 限制速度大小
*/
void dlo::OdomNode::NormalizeVelocity() {
    Eigen::Vector3d v_body = this->lidarState.qwb.inverse() * this->lidarState.Vw;
    v_body[0] = xio::maxMin(v_body[0], 1, -1);
    v_body[1] = xio::maxMin(v_body[1], 1, -1);
    v_body[2] = xio::maxMin(v_body[2], 1, -1);
    this->lidarState.Vw = this->lidarState.qwb * v_body;
}

/**
 * Get Next Pose
 **/

void dlo::OdomNode::getNextPose() {
  xio::NavState tmpState = imuPreinteg_Opt_->Predict(lidarState);
  Eigen::Matrix4d guess_SE3 = Eigen::Matrix4d::Identity();
  guess_SE3.block<3, 3>(0, 0) = tmpState.qwb.matrix();  //this->rotq.matrix().cast<double>();
  guess_SE3.block<3, 1>(0, 3) = tmpState.Pwb;//this->pose.cast<double>()
  this->T_s2s = guess_SE3.cast<float>();
  // Get current global submap
  this->getSubmapKeyframes();

  if (this->submap_hasChanged) {

    // Set the current global submap as the target cloud
    this->gicp.setInputTarget(this->submap_cloud);

    // Set target cloud's normals as submap normals
    this->gicp.setTargetCovariances( this->submap_normals );
  }

  pcl::PointCloud<PointType>::Ptr aligned (new pcl::PointCloud<PointType>);
  // Align with current submap with global S2S transformation as initial guess
  this->gicp.align(*aligned, this->T_s2s);  //this->T_s2s

  // Get final transformation in global frame
  this->T = this->gicp.getFinalTransformation();

  // Update next global pose
  // Both source and target clouds are in the global frame now, so tranformation is global
  this->propagateS2M();

}


/**
 * Propagate S2M Alignment
 **/
void dlo::OdomNode::propagateS2M() {

  this->pose   << this->T(0,3), this->T(1,3), this->T(2,3);
  this->rotSO3 << this->T(0,0), this->T(0,1), this->T(0,2),
                  this->T(1,0), this->T(1,1), this->T(1,2),
                  this->T(2,0), this->T(2,1), this->T(2,2);

  Eigen::Quaternionf q(this->rotSO3);

  // Normalize quaternion
  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());
  q.w() /= norm; q.x() /= norm; q.y() /= norm; q.z() /= norm;
  this->rotq = q;

}


/**
 * Transform Current Scan
 * 点云变换到全局,用来更新keyframe
 **/
void dlo::OdomNode::transformCurrentScan() {
  this->current_scan_t = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  pcl::transformPointCloud (*this->current_scan, *this->current_scan_t, this->T);
}


/**
 * Compute Metrics
 **/
void dlo::OdomNode::computeMetrics() {
  this->computeSpaciousness();
}


/**
 * Compute Spaciousness of Current Scan
 **/
void dlo::OdomNode::computeSpaciousness() {

  // compute range of points
  std::vector<float> ds;

  for (int i = 0; i <= this->current_scan->points.size(); i++) {
    float d = std::sqrt(pow(this->current_scan->points[i].x, 2) + pow(this->current_scan->points[i].y, 2) + pow(this->current_scan->points[i].z, 2));
    ds.push_back(d);
  }

  // median
  std::nth_element(ds.begin(), ds.begin() + ds.size()/2, ds.end());
  float median_curr = ds[ds.size()/2];
  static float median_prev = median_curr;
  float median_lpf = 0.95*median_prev + 0.05*median_curr;
  median_prev = median_lpf;

  // push
  this->metrics.spaciousness.push_back( median_lpf );

}


/**
 * Convex Hull of Keyframes
 **/
void dlo::OdomNode::computeConvexHull() {

  // at least 4 keyframes for convex hull
  if (this->num_keyframes < 4) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);

  for (const auto& k : this->keyframes) {
    PointType pt;
    pt.x = k.first.first[0];
    pt.y = k.first.first[1];
    pt.z = k.first.first[2];
    cloud->push_back(pt);
  }

  // calculate the convex hull of the point cloud
  this->convex_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the convex hull
  pcl::PointCloud<PointType>::Ptr convex_points = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->convex_hull.reconstruct(*convex_points);

  pcl::PointIndices::Ptr convex_hull_point_idx = pcl::PointIndices::Ptr (new pcl::PointIndices);
  this->convex_hull.getHullPointIndices(*convex_hull_point_idx);

  this->keyframe_convex.clear();
  for (int i=0; i<convex_hull_point_idx->indices.size(); ++i) {
    this->keyframe_convex.push_back(convex_hull_point_idx->indices[i]);
  }

}


/**
 * Concave Hull of Keyframes
 **/
void dlo::OdomNode::computeConcaveHull() {

  // at least 5 keyframes for concave hull
  if (this->num_keyframes < 5) {
    return;
  }

  // create a pointcloud with points at keyframes
  pcl::PointCloud<PointType>::Ptr cloud = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);

  for (const auto& k : this->keyframes) {
    PointType pt;
    pt.x = k.first.first[0];
    pt.y = k.first.first[1];
    pt.z = k.first.first[2];
    cloud->push_back(pt);
  }

  // calculate the concave hull of the point cloud
  this->concave_hull.setInputCloud(cloud);

  // get the indices of the keyframes on the concave hull
  pcl::PointCloud<PointType>::Ptr concave_points = pcl::PointCloud<PointType>::Ptr (new pcl::PointCloud<PointType>);
  this->concave_hull.reconstruct(*concave_points);

  pcl::PointIndices::Ptr concave_hull_point_idx = pcl::PointIndices::Ptr (new pcl::PointIndices);
  this->concave_hull.getHullPointIndices(*concave_hull_point_idx);

  this->keyframe_concave.clear();
  for (int i=0; i<concave_hull_point_idx->indices.size(); ++i) {
    this->keyframe_concave.push_back(concave_hull_point_idx->indices[i]);
  }

}


/**
 * Update keyframes
 **/
void dlo::OdomNode::updateKeyframes() {

  // transform point cloud
  this->transformCurrentScan();

  // calculate difference in pose and rotation to all poses in trajectory
  float closest_d = std::numeric_limits<float>::infinity();
  int closest_idx = 0;
  int keyframes_idx = 0;

  int num_nearby = 0;

  for (const auto& k : this->keyframes) {

    // calculate distance between current pose and pose in keyframes
    float delta_d = sqrt( pow(this->pose[0] - k.first.first[0], 2) + pow(this->pose[1] - k.first.first[1], 2) + pow(this->pose[2] - k.first.first[2], 2) );

    // count the number nearby current pose
    if (delta_d <= this->keyframe_thresh_dist_ * 1.5){
      ++num_nearby;
    }

    // store into variable
    if (delta_d < closest_d) {
      closest_d = delta_d;
      closest_idx = keyframes_idx;
    }

    keyframes_idx++;

  }

  // get closest pose and corresponding rotation
  Eigen::Vector3f closest_pose = this->keyframes[closest_idx].first.first;
  Eigen::Quaternionf closest_pose_r = this->keyframes[closest_idx].first.second;

  // calculate distance between current pose and closest pose from above
  float dd = sqrt( pow(this->pose[0] - closest_pose[0], 2) + pow(this->pose[1] - closest_pose[1], 2) + pow(this->pose[2] - closest_pose[2], 2) );

  // calculate difference in orientation，这里用转置是不是更快
  Eigen::Quaternionf dq = this->rotq * (closest_pose_r.inverse());

  float theta_rad = 2. * atan2(sqrt( pow(dq.x(), 2) + pow(dq.y(), 2) + pow(dq.z(), 2) ), dq.w());
  float theta_deg = theta_rad * (180.0/M_PI);

  // update keyframe
  bool newKeyframe = false;

  if (abs(dd) > this->keyframe_thresh_dist_ || abs(theta_deg) > this->keyframe_thresh_rot_) {
    newKeyframe = true;
  }
  if (abs(dd) <= this->keyframe_thresh_dist_) {
    newKeyframe = false;
  }
  if (abs(dd) <= this->keyframe_thresh_dist_ && abs(theta_deg) > this->keyframe_thresh_rot_ && num_nearby <= 1) {
    newKeyframe = true;
  }

  if (newKeyframe) {

    ++this->num_keyframes;

    // voxelization for submap
    if (this->vf_submap_use_) {
      this->vf_submap.setInputCloud(this->current_scan_t);
      this->vf_submap.filter(*this->current_scan_t);
    }

    // update keyframe vector
    this->keyframes.push_back(std::make_pair(std::make_pair(this->pose, this->rotq), this->current_scan_t));
    *this->keyframe_cloud = *this->current_scan_t;

    //更新协方差矩阵
    const nano_gicp::CovarianceList raw_covariances = this->gicp.getSourceCovariances();
    nano_gicp::CovarianceList transformed_covariances(raw_covariances.size());
    Eigen::Matrix4d Td = this->T.cast<double>();
    std::transform(raw_covariances.begin(), raw_covariances.end(), transformed_covariances.begin(),
                   [&Td](Eigen::Matrix4d cov) { return Td * cov * Td.transpose(); });
    this->keyframe_normals.push_back(transformed_covariances);

    this->publish_keyframe_thread = std::thread( &dlo::OdomNode::publishKeyframe, this );
    this->publish_keyframe_thread.detach();

  }

}


/**
 * Set Adaptive Parameters
 **/
void dlo::OdomNode::setAdaptiveParams() {

  // Set Keyframe Thresh from Spaciousness Metric
  if (this->metrics.spaciousness.back() > 20.0){
    this->keyframe_thresh_dist_ = 10.0;
  } else if (this->metrics.spaciousness.back() > 10.0 && this->metrics.spaciousness.back() <= 20.0) {
    this->keyframe_thresh_dist_ = 5.0;
  } else if (this->metrics.spaciousness.back() > 5.0 && this->metrics.spaciousness.back() <= 10.0) {
    this->keyframe_thresh_dist_ = 1.0;
  } else if (this->metrics.spaciousness.back() <= 5.0) {
    this->keyframe_thresh_dist_ = 0.5;
  }

  // set concave hull alpha
  this->concave_hull.setAlpha(this->keyframe_thresh_dist_);

}


/**
 * Push Submap Keyframe Indices
 **/
void dlo::OdomNode::pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames) {

  // make sure dists is not empty
  if (!dists.size()) { return; }

  // maintain max heap of at most k elements
  std::priority_queue<float> pq;

  for (auto d : dists) {
    if (pq.size() >= k && pq.top() > d) {
      pq.push(d);
      pq.pop();
    } else if (pq.size() < k) {
      pq.push(d);
    }
  }

  // get the kth smallest element, which should be at the top of the heap
  float kth_element = pq.top();

  // get all elements smaller or equal to the kth smallest element
  for (int i = 0; i < dists.size(); ++i) {
    if (dists[i] <= kth_element)
      this->submap_kf_idx_curr.push_back(frames[i]);
  }

}


/**
 * Get Submap using Nearest Neighbor Keyframes
 **/
void dlo::OdomNode::getSubmapKeyframes() {

  // clear vector of keyframe indices to use for submap
  this->submap_kf_idx_curr.clear();

  //
  // TOP K NEAREST NEIGHBORS FROM ALL KEYFRAMES
  //

  // calculate distance between current pose and poses in keyframe set
  std::vector<float> ds;
  std::vector<int> keyframe_nn; int i=0;
  Eigen::Vector3f curr_pose = this->T_s2s.block(0,3,3,1);

  for (const auto& k : this->keyframes) {
    float d = sqrt( pow(curr_pose[0] - k.first.first[0], 2) + pow(curr_pose[1] - k.first.first[1], 2) + pow(curr_pose[2] - k.first.first[2], 2) );
    ds.push_back(d);
    keyframe_nn.push_back(i); i++;
  }

  // get indices for top K nearest neighbor keyframe poses
  this->pushSubmapIndices(ds, this->submap_knn_, keyframe_nn);

  //
  // TOP K NEAREST NEIGHBORS FROM CONVEX HULL
  //

  // get convex hull indices
  this->computeConvexHull();

  // get distances for each keyframe on convex hull
  std::vector<float> convex_ds;
  for (const auto& c : this->keyframe_convex) {
    convex_ds.push_back(ds[c]);
  }

  // get indicies for top kNN for convex hull
  this->pushSubmapIndices(convex_ds, this->submap_kcv_, this->keyframe_convex);

  //
  // TOP K NEAREST NEIGHBORS FROM CONCAVE HULL
  //

  // get concave hull indices
  this->computeConcaveHull();

  // get distances for each keyframe on concave hull
  std::vector<float> concave_ds;
  for (const auto& c : this->keyframe_concave) {
    concave_ds.push_back(ds[c]);
  }

  // get indicies for top kNN for convex hull
  this->pushSubmapIndices(concave_ds, this->submap_kcc_, this->keyframe_concave);

  //
  // BUILD SUBMAP
  //

  // concatenate all submap clouds and normals
  //去重
  std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  auto last = std::unique(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  this->submap_kf_idx_curr.erase(last, this->submap_kf_idx_curr.end());

  // sort current and previous submap kf list of indices
  //排序以比较是否相等
  std::sort(this->submap_kf_idx_curr.begin(), this->submap_kf_idx_curr.end());
  std::sort(this->submap_kf_idx_prev.begin(), this->submap_kf_idx_prev.end());

  // check if submap has changed from previous iteration
  if (this->submap_kf_idx_curr == this->submap_kf_idx_prev){
    this->submap_hasChanged = false;
  } else {
    this->submap_hasChanged = true;

    // reinitialize submap cloud, normals
    pcl::PointCloud<PointType>::Ptr submap_cloud_ (boost::make_shared<pcl::PointCloud<PointType>>());
    this->submap_normals.clear();

    for (auto k : this->submap_kf_idx_curr) {

      // create current submap cloud
      *submap_cloud_ += *this->keyframes[k].second;

      // grab corresponding submap cloud's normals
      this->submap_normals.insert( std::end(this->submap_normals), std::begin(this->keyframe_normals[k]), std::end(this->keyframe_normals[k]) );
    }

    this->submap_cloud = submap_cloud_;
    this->submap_kf_idx_prev = this->submap_kf_idx_curr;
  }

}


/**
 * Debug Statements
 **/

void dlo::OdomNode::debug() {

  // Total length traversed
  double length_traversed = 0.;
  Eigen::Vector3f p_curr = Eigen::Vector3f(0., 0., 0.);
  Eigen::Vector3f p_prev = Eigen::Vector3f(0., 0., 0.);
  for (const auto& t : this->trajectory) {
    if (p_prev == Eigen::Vector3f(0., 0., 0.)) {
      p_prev = t.first;
      continue;
    }
    p_curr = t.first;
    double l = sqrt(pow(p_curr[0] - p_prev[0], 2) + pow(p_curr[1] - p_prev[1], 2) + pow(p_curr[2] - p_prev[2], 2));

    if (l >= 0.05) {
      length_traversed += l;
      p_prev = p_curr;
    }
  }

  if (length_traversed == 0) {
    this->publish_keyframe_thread = std::thread( &dlo::OdomNode::publishKeyframe, this );
    this->publish_keyframe_thread.detach();
  }

  // Average computation time
  double avg_comp_time = std::accumulate(this->comp_times.begin(), this->comp_times.end(), 0.0) / this->comp_times.size();

  // RAM Usage
  double vm_usage = 0.0;
  double resident_set = 0.0;
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string num_threads, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
              >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
              >> utime >> stime >> cutime >> cstime >> priority >> nice
              >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
  stat_stream.close();
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;

  // CPU Usage
  struct tms timeSample;
  clock_t now;
  double cpu_percent;
  now = times(&timeSample);
  if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
      timeSample.tms_utime < this->lastUserCPU) {
      cpu_percent = -1.0;
  } else {
      cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
      cpu_percent /= (now - this->lastCPU);
      cpu_percent /= this->numProcessors;
      cpu_percent *= 100.;
  }
  this->lastCPU = now;
  this->lastSysCPU = timeSample.tms_stime;
  this->lastUserCPU = timeSample.tms_utime;
  this->cpu_percents.push_back(cpu_percent);
  double avg_cpu_usage = std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

  // Print to terminal
  printf("\033[2J\033[1;1H");

  std::cout << std::endl << "==== Direct LiDAR Odometry v" << this->version_ << " ====" << std::endl;

  if (!this->cpu_type.empty()) {
    std::cout << std::endl << this->cpu_type << " x " << this->numProcessors << std::endl;
  }

  std::cout << std::endl << std::setprecision(4) << std::fixed;
  std::cout << "Position    [xyz]  :: " << this->pose[0] << " " << this->pose[1] << " " << this->pose[2] << std::endl;
  std::cout << "Orientation [wxyz] :: " << this->rotq.w() << " " << this->rotq.x() << " " << this->rotq.y() << " " << this->rotq.z() << std::endl;
  std::cout << "Distance Traveled  :: " << length_traversed << " meters" << std::endl;
  std::cout << "Distance to Origin :: " << sqrt(pow(this->pose[0]-this->origin[0],2) + pow(this->pose[1]-this->origin[1],2) + pow(this->pose[2]-this->origin[2],2)) << " meters" << std::endl;

  std::cout << std::endl << std::right << std::setprecision(2) << std::fixed;
  int vecsize = comp_times.size();
  std::cout << "Whole computation Time :: " << std::setfill(' ') << std::setw(6) << this->comp_times[vecsize-1]*1000. << " ms    // Avg: " << std::setw(5) << avg_comp_time*1000. << std::endl;
  std::cout << "integ+gicp Computation Time :: " << std::setfill(' ') << std::setw(6) << this->comp_times[vecsize-2]*1000. << " ms    // integ: " << std::setw(5) << this->comp_times[vecsize-3]*1000. << std::endl;
  std::cout<< "deskew?: " << this->deskew_ <<", total size: "<<this->totalPointNums<<"deskew size: "<<this->deskewNums<<"percentage"<<(double)this->deskewNums/this->totalPointNums<< std::endl;
  std::cout << "Cores Utilized   :: " << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: " << std::setw(5) << (avg_cpu_usage/100.) * this->numProcessors << std::endl;
  std::cout << "CPU Load         :: " << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: " << std::setw(5) << avg_cpu_usage << std::endl;
  std::cout << "RAM Allocation   :: " << std::setfill(' ') << std::setw(6) << resident_set/1000. << " MB    // VSZ: " << vm_usage/1000. << " MB" << std::endl;

}
