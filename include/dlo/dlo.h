/************************************************************
 *
 * Copyright (c) 2022, University of California, Los Angeles
 *
 * Authors: Kenny J. Chen, Brett T. Lopez
 * Contact: kennyjchen@ucla.edu, btlopez@ucla.edu
 *
 ***********************************************************/

#include <atomic>
#include <fstream>
#include <iomanip>
#include <ios>
#include <iostream>
#include <mutex>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sys/times.h>
#include <sys/vtimes.h>
#include <thread>
#include<condition_variable>

#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <ros/ros.h>

// BOOST
#include <boost/format.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/extract_indices.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <xio/save_pcd.h>
#include <nano_gicp/nano_gicp.hpp>

// typedef pcl::PointXYZI PointType;

namespace xio {
  struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    union {
      std::uint32_t t; // time since beginning of scan in nanoseconds
      float time; // time since beginning of scan in seconds
      std::uint32_t offset_time;
      double timestamp; // absolute timestamp in seconds
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(xio::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, t, t)
                                 (float, time, time)
                                 (std::uint32_t, offset_time, offset_time)
                                 (double, timestamp, timestamp))

typedef xio::Point PointType;

struct LivoxPoint {
  LivoxPoint(): data{0.f, 0.f, 0.f, 1.f} {}
  PCL_ADD_POINT4D;
  float intensity; // intensity
  std::uint32_t offset_time; // LIVOX: time from beginning of scan in nanoseconds
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, offset_time, offset_time))

namespace dlo {

  class OdomNode;
  class MapNode;

}
