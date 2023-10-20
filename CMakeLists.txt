#############################################################
#                                                           #
# Copyright (c) 2022, University of California, Los Angeles #
#                                                           #
# Authors: Kenny J. Chen, Brett T. Lopez                    #
# Contact: kennyjchen@ucla.edu, btlopez@ucla.edu            #
#                                                           #
#############################################################

cmake_minimum_required(VERSION 3.10.0)
project(xio)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")

find_package( PCL REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS})

find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(/usr/include/glog)
include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)

set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)
find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  pcl_ros
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES
  save_pcd.srv
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    pcl_ros
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    nano_gicp
    nanoflann
  DEPENDS PCL GTSAM
)

find_package(GTSAM REQUIRED)
set(g2o_DIR /usr/local/lib/cmake/g2o)
find_package(g2o REQUIRED)
SET(G2O_LIBS /usr/local/lib/libg2o_cli.so /usr/local/lib/libg2o_ext_freeglut_minimal.so /usr/local/lib/libg2o_simulator.so /usr/local/lib/libg2o_solver_slam2d_linear.so /usr/local/lib/libg2o_types_icp.so /usr/local/lib/libg2o_types_slam2d.so /usr/local/lib/libg2o_core.so /usr/local/lib/libg2o_interface.so /usr/local/lib/libg2o_solver_csparse.so /usr/local/lib/libg2o_solver_structure_only.so /usr/local/lib/libg2o_types_sba.so /usr/local/lib/libg2o_types_slam3d.so /usr/local/lib/libg2o_csparse_extension.so /usr/local/lib/libg2o_opengl_helper.so /usr/local/lib/libg2o_solver_dense.so /usr/local/lib/libg2o_stuff.so /usr/local/lib/libg2o_types_sclam2d.so /usr/local/lib/libg2o_parser.so /usr/local/lib/libg2o_solver_pcg.so /usr/local/lib/libg2o_types_data.so /usr/local/lib/libg2o_types_sim3.so cxsparse )
find_package(Sophus REQUIRED)
find_package(FMT REQUIRED)

include_directories(
  include ${catkin_INCLUDE_DIRS} 
  ${GTSAM_INCLUDE_DIR} 
  ${PCL_INCLUDE_DIRS}  
  ${g2o_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS})

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

# NanoFLANN
add_library(nanoflann STATIC
  src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
target_include_directories(nanoflann PUBLIC include ${PCL_INCLUDE_DIRS})

# NanoGICP
add_library(nano_gicp STATIC
  src/nano_gicp/lsq_registration.cc
  src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
target_include_directories(nano_gicp PUBLIC include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# Odometry Node
add_executable(dlo_odom_node src/dlo/odom_node.cc src/dlo/odom.cc)
add_dependencies(dlo_odom_node ${catkin_EXPORTED_TARGETS})
target_compile_options(dlo_odom_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlo_odom_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} 
                                          ${OpenMP_LIBS} Threads::Threads nano_gicp ${G2O_LIBS}
                                          ${Sophus_LIBRARIES} fmt::fmt
                                          /usr/lib/x86_64-linux-gnu/libglog.so)

# Mapping Node
add_executable (dlo_map_node src/dlo/map_node.cc src/dlo/map.cc)
add_dependencies(dlo_map_node ${catkin_EXPORTED_TARGETS})
target_compile_options(dlo_map_node PRIVATE ${OpenMP_FLAGS})
target_link_libraries(dlo_map_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads)