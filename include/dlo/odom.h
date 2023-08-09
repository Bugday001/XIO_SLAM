/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include "dlo/dlo.h"
#include "g2o_struct.h"
#include "imuPreintFun.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

class dlo::OdomNode {

public:

  OdomNode(ros::NodeHandle node_handle);
  ~OdomNode();

  static void abort() {
    abort_ = true;
  }

  void start();
  void stop();

private:

  void abortTimerCB(const ros::TimerEvent& e);
  void icpCB(const sensor_msgs::PointCloud2ConstPtr& pc);
  void imuCB(const sensor_msgs::Imu::ConstPtr& imu);
  void gtCB(const nav_msgs::OdometryConstPtr &PointRes);

  void getParams();

  void publishToROS();
  void publishImuOdom();
  void publishPose(const ros::TimerEvent& e);
  void publishTransform();
  void publishKeyframe();
  void publishCloud();

  void preprocessPoints(const sensor_msgs::PointCloud2ConstPtr& pc);
  void deskewPointcloud();
  void initializeInputTarget();
  void setInputSources();

  void initializeDLO();
  void gravityAlign();

  void imuOptPreint(double end_time);
  void getNextPose();
  void imuPreintegration();
  void Optimize();
  void NormalizeVelocity();

  void propagateS2M();
  sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in);

  void setAdaptiveParams();

  void computeMetrics();
  void computeSpaciousness();

  void transformCurrentScan();
  void updateKeyframes();
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
  void getSubmapKeyframes();

  void debug();

  double first_imu_time;

  ros::NodeHandle nh;
  ros::Timer abort_timer;

  ros::Subscriber icp_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber gt_sub;

  ros::Publisher odom_pub;
  ros::Publisher pose_pub;
  ros::Publisher keyframe_pub;
  ros::Publisher kf_pub;
  ros::Publisher cur_cloud_t_pub, imu_odom_pub_;

  ros::Timer publish_timer;

  Eigen::Vector3f origin;
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>, pcl::PointCloud<PointType>::Ptr>> keyframes;
  std::vector<nano_gicp::CovarianceList> keyframe_normals;

  std::atomic<bool> dlo_initialized;
  std::atomic<bool> imu_calibrated;
  std::atomic<int> deskew_size;

  std::string odom_frame;
  std::string child_frame;

  pcl::PointCloud<PointType>::Ptr original_scan;
  pcl::PointCloud<PointType>::Ptr current_scan;
  pcl::PointCloud<PointType>::Ptr current_scan_t;
  pcl::PointCloud<PointType>::Ptr deskewed_scan;
  pcl::PointCloud<PointType>::Ptr keyframe_cloud;

  bool deskew_status;
  bool first_valid_scan;
  bool deskew_;
  std::vector<std::pair<xio::NavState, double>> imu_states_;
  long deskewNums, totalPointNums;

  double scan_median_stamp;
  double prev_scan_median_stamp;

  int num_keyframes;  

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  pcl::PointCloud<PointType>::Ptr submap_cloud;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> submap_normals;

  std::vector<int> submap_kf_idx_curr;
  std::vector<int> submap_kf_idx_prev;
  std::atomic<bool> submap_hasChanged;

  pcl::PointCloud<PointType>::Ptr source_cloud;
  pcl::PointCloud<PointType>::Ptr target_cloud;

  ros::Time scan_stamp;

  double curr_frame_stamp;
  double prev_frame_stamp;
  std::vector<double> comp_times;

  nano_gicp::NanoGICP<PointType, PointType> gicp;

  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> vf_scan;
  pcl::VoxelGrid<PointType> vf_submap;

  nav_msgs::Odometry odom;
  nav_msgs::Odometry kf;

  geometry_msgs::PoseStamped pose_ros;
  geometry_msgs::Point uwbPoint;

  Eigen::Matrix4f T;
  Eigen::Matrix4f T_s2s;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f pose_s2s;
  Eigen::Matrix3f rotSO3_s2s;
  Eigen::Quaternionf rotq_s2s;

  Eigen::Vector3f pose;
  Eigen::Matrix3f rotSO3;
  Eigen::Quaternionf rotq;

  Eigen::Matrix4f imu_SE3;

  Eigen::Vector3d prev_point_;

  struct XYZd {
    double x;
    double y;
    double z;
  };

  struct ImuBias {
    XYZd gyro;
    XYZd accel;
  };

  ImuBias imu_bias;

  struct ImuMeas {
    double stamp;
    XYZd ang_vel;
    XYZd lin_accel;
  };


  // Sensor Type
  xio::SensorType sensor;
  //imu
  ros::Time imu_stamp;
  ImuMeas imu_meas;
  std::vector<double> extRotV;
  std::vector<double> extRPYV;
  std::vector<double> extTransV;
  Eigen::Matrix3d extRot;
  Eigen::Quaterniond extQRPY;
  int imuType;
  boost::circular_buffer<ImuMeas> imu_buffer;
  std::deque<sensor_msgs::Imu> imuQueImu, imuQueOpt;

  xio::NavState xioState, lidarState, preLidarState;
  double lastImuT_imu;
  double currentCorrectionTime;  //newest lidar odom time
  std::shared_ptr<xio::IMUPreintegration> imuPreinteg_ = nullptr, imuPreinteg_Opt_ = nullptr;
  //opt
  bool imuOptInitialized = false;
  Eigen::Matrix<double, 15, 15> prior_info_ = Eigen::Matrix<double, 15, 15>::Identity();
  double lastImuT_opt;
  double delta_t = 0;
  bool debugVerbose;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  struct Metrics {
    std::vector<float> spaciousness;
  };

  Metrics metrics;

  static std::atomic<bool> abort_;
  std::atomic<bool> stop_publish_thread;
  std::atomic<bool> stop_publish_keyframe_thread;
  std::atomic<bool> stop_metrics_thread;
  std::atomic<bool> stop_debug_thread;

  std::thread publish_thread;
  std::thread publish_keyframe_thread;
  std::thread metrics_thread;
  std::thread debug_thread;

  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  std::string cpu_type;
  std::vector<double> cpu_percents;
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;

  // Parameters
  std::string version_;

  bool gravity_align_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool initial_pose_use_;
  Eigen::Vector3f initial_position_;
  Eigen::Quaternionf initial_orientation_;

  bool crop_use_;
  double crop_size_;

  bool vf_scan_use_;
  double vf_scan_res_;

  bool vf_submap_use_;
  double vf_submap_res_;

  bool adaptive_params_use_;

  bool imu_use_;
  int imu_calib_time_;
  int imu_buffer_size_;

  int gicp_min_num_points_;

  int gicps2s_k_correspondences_;
  double gicps2s_max_corr_dist_;
  int gicps2s_max_iter_;
  double gicps2s_transformation_ep_;
  double gicps2s_euclidean_fitness_ep_;
  int gicps2s_ransac_iter_;
  double gicps2s_ransac_inlier_thresh_;

  int gicps2m_k_correspondences_;
  double gicps2m_max_corr_dist_;
  int gicps2m_max_iter_;
  double gicps2m_transformation_ep_;
  double gicps2m_euclidean_fitness_ep_;
  int gicps2m_ransac_iter_;
  double gicps2m_ransac_inlier_thresh_;

};
