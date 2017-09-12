#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>

#include "stopwatch.h"
#include <fstream>


bool initialized = false;
tf2_ros::Buffer* tf_buffer;
tf2_ros::TransformListener* tf_listener;
Stopwatch global_time;
std::ofstream datafile("nanomap_data_1000.txt");
int point_cloud_count = 0;

void PointCloudCallback(const sensor_msgs::PointCloud2& msg) {
  Stopwatch sw;
  sw.Start();
  geometry_msgs::TransformStamped transform_world_sensor;
  try {
    transform_world_sensor = tf_buffer->lookupTransform("world", msg.header.frame_id, msg.header.stamp);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  
  float insertion_time = sw.ElapsedMillis();
  
  sw.Start();

  sw.Stop();
  float distance_update_time = sw.ElapsedMillis();
  std::cout << "distance_update_time: " << distance_update_time << std::endl;

  sw.Start();
  int num_samples = 10;
  float rad = 5;
  float delta = rad/(num_samples-1);
  for (float x = -rad; x < rad; x += delta) {
    for (float y = -rad; y < rad; y += delta) {
      for (float z = -rad; z < rad; z += delta) {
	//octomap::point3d query_point = sensor_origin + octomap::point3d(x, y, z);
	//distmap.getDistanceAndClosestObstacle(query_point, distance, closestObst);
      }
    }
  }
  sw.Stop();
  float sample_time = sw.ElapsedMillis();

  datafile << point_cloud_count << "," << msg.header.seq << "," << global_time.ElapsedMillis() << "," << insertion_time << "," << distance_update_time << "," << sample_time << std::endl;
  std::cout << "Processed point cloud: " << point_cloud_count << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nanomap_benchmarker");
    datafile << "point_cloud_count, sequence_number, time(ms), insertion_time(ms), distance_update_time(ms), sample_time(ms)" << std::endl;

    ros::NodeHandle nh;
    tf_buffer = new tf2_ros::Buffer();
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    ros::Subscriber pcl_sub = nh.subscribe("/flight/r200/points_xyz", 100, &PointCloudCallback);
    global_time.Start();
    ros::spin();

    return 0;
}
