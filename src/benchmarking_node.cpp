#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>

#include "stopwatch.h"
#include <fstream>
#include <nav_msgs/Path.h>

#include "nanomap.h"


NanoMap nanomap;

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

  pcl::PCLPointCloud2 cloud2_rdf;
  pcl_conversions::toPCL(msg, cloud2_rdf);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rdf(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud2_rdf,*cloud_rdf);
  NanoMapTime nm_time(msg.header.stamp.sec, msg.header.stamp.nsec);
  nanomap.AddPointCloud(cloud_rdf, nm_time, msg.header.seq);

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

void PoseCallback(geometry_msgs::PoseStamped const& pose) {
  Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
  Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
  NanoMapPose nm_pose(pos, quat, nm_time);
  nanomap.AddPose(nm_pose);
}

void SmoothedPosesCallback(nav_msgs::Path path) {
  std::vector<NanoMapPose> smoothed_path_vector;
  size_t path_size = path.poses.size();
  for (size_t i = 0; i < path_size; i++) {
    geometry_msgs::PoseStamped pose = path.poses.at(i);
    Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
    NanoMapPose nm_pose(pos, quat, nm_time);
    smoothed_path_vector.push_back(nm_pose);
  }
  nanomap.AddPoseUpdates(smoothed_path_vector);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "nanomap_benchmarker");
    
    Matrix3 K;
    K << 205.27, 0.0, 160.0, 0.0, 205.27, 120.0, 0.0, 0.0, 1.0;
    nanomap.SetCameraInfo(1.0, 320.0, 240.0, K);
    nanomap.SetSensorRange(20.0);
    nanomap.SetNumDepthImageHistory(150);
    Matrix3 body_to_rdf;
    body_to_rdf << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    nanomap.SetBodyToRdf(body_to_rdf);

    datafile << "point_cloud_count, sequence_number, time(ms), insertion_time(ms), distance_update_time(ms), sample_time(ms)" << std::endl;

    ros::NodeHandle nh;
    tf_buffer = new tf2_ros::Buffer();
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    ros::Subscriber pcl_sub = nh.subscribe("/flight/r200/points_xyz", 100, &PointCloudCallback);
    ros::Subscriber pose_updates_sub = nh.subscribe("/samros/keyposes", 100, &SmoothedPosesCallback);
    ros::Subscriber pose_sub = nh.subscribe("/pose", 100, &PoseCallback);
    global_time.Start();
    ros::spin();

    return 0;
}
