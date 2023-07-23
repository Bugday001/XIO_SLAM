cmake_minimum_required(VERSION 3.0.2)
project(XIO)

set(CMAKE_CXX_FLAGS "-std=c++14")
find_package(Boost REQUIRED COMPONENTS timer)
find_package(PCL REQUIRED)
find_package(catkin REQUIRED
    pcl_ros
    pcl_conversions
  )

catkin_package(
  DEPENDS PCL GTSAM
)

find_package(GTSAM REQUIRED)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
        ${GTSAM_LIBRARY_DIRS}
)

add_executable(imuPreintegration_node src/imuPreintegration.cpp)

target_link_libraries(imuPreintegration_node
  ${catkin_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES} gtsam
)