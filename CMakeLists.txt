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
  message_generation
  sensor_msgs
)

find_package( Eigen3 REQUIRED )
find_package(OpenCV REQUIRED)

include_directories ( src )
include_directories( ${EIGEN3_INCLUDE_DIR} )

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions("-std=c++11")

find_package(orocos_kdl REQUIRED)
find_library(OROCOS_KDL orocos-kdl)
set(orocos_kdl_LIBRARIES ${OROCOS_KDL})


add_library( nanomap_ros src/nanomap.cc src/pose_manager.cc src/structured_point_cloud_chain.cc src/fov_evaluator.cc src/nanomap_visualizer.cc src/pcl.cc )

target_link_libraries( nanomap_ros ${catkin_LIBRARIES} orocos-kdl )

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test
    test/test_nanomap.cpp
  )

  target_link_libraries(${PROJECT_NAME}-test
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GCOV_LIBRARY}
  )
endif() # CATKIN_ENABLE_TESTING

add_executable( benchmarking_node example_nodes/benchmarking_node.cpp )
target_link_libraries( benchmarking_node nanomap_ros ${catkin_LIBRARIES} orocos-kdl )

add_executable( visualization_node example_nodes/visualization_node.cpp )
target_link_libraries( visualization_node nanomap_ros ${catkin_LIBRARIES} orocos-kdl )