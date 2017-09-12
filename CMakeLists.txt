cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")
project(nanomap_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  visualization_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  cv_bridge
  image_transport
  acl_fsw
  message_generation
  sensor_msgs
  mavros_msgs
  pcl_ros
  fla_msgs
  fla_utils
)

find_package( Eigen3 REQUIRED )
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories ( src )
include_directories( ${EIGEN3_INCLUDE_DIR} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions("-std=c++11")

find_package(orocos_kdl REQUIRED)
find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})


add_library( nanomap_ros src/nanomap.cc src/pose_manager.cc src/structured_point_cloud_chain.cc src/fov_evaluator.cc src/nanomap_visualizer.cc)

target_link_libraries( nanomap_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES} orocos-kdl)

