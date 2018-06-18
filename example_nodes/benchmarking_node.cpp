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

  PointCloud<PointXYZ>::Ptr cloud_rdf(new PointCloud<PointXYZ>);
  fromSensorMsgsPointCloud2(msg,*cloud_rdf);
  NanoMapTime nm_time(msg.header.stamp.sec, msg.header.stamp.nsec);
  nanomap.AddPointCloud(cloud_rdf, nm_time, msg.header.seq);

  float insertion_time = sw.ElapsedMillis();
  std::cout << "insertion_time: " << insertion_time << std::endl;
 
  sw.Start();

  sw.Stop();
  float distance_update_time = sw.ElapsedMillis();
  //std::cout << "distance_update_time: " << distance_update_time << std::endl;

  sw.Start();
  int num_samples = 10;
  float rad = 5.0;
  float delta = 2*rad/(num_samples-1)-.0001; // subtraction needed to get floating point happy in loop
  
  NanoMapKnnArgs args;
  args.axis_aligned_linear_covariance = Vector3(0.1, 0.1, 0.1);
  args.early_exit = false;
  args.query_point_current_body_frame = Vector3(3.0, 0.0, 0.0);
  //int ni = 0;
  for (float x = -rad; x <= rad; x =x+delta) {
    for (float y = -rad; y <= rad; y =y+delta) {
      for (float z = -rad; z <= rad; z =z+delta) {
         //ni++;
         args.query_point_current_body_frame = Vector3(x, y, z);
         NanoMapKnnReply reply = nanomap.KnnQuery(args);	
      }
    }
  }
  sw.Stop();
  float sample_time = sw.ElapsedMillis();
  std::cout << "sample_time: " << sample_time << std::endl;
  //std::cout << "num queries " << ni << std::endl;

  point_cloud_count++;
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
    nanomap.SetCameraInfo(4.0, 320.0, 240.0, K); // this K matrix was for binned, but the actual data is not binned
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
