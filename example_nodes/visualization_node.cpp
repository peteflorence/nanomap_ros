#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>

#include "nanomap.h"
#include "nanomap_visualizer.h"


class NanoMapNode {
  public:
  NanoMapNode() : nh("~"){
    nanomap_visualizer.Initialize(nh);

    ros::NodeHandle nh;
    NanoMapVisualizer nanomap_visualizer;
    nanomap_visualizer.Initialize(nh);

    Matrix3 K;
    K << 205.27, 0.0, 160.0, 0.0, 205.27, 120.0, 0.0, 0.0, 1.0;
    nanomap.SetCameraInfo(4.0, 320.0, 240.0, K); // this K matrix was for binned, but the actual data is not binned
    nanomap.SetSensorRange(20.0);
    nanomap.SetNumDepthImageHistory(150);
    Matrix3 body_to_rdf;
    body_to_rdf << 0, -1, 0, 0, 0, -1, 1, 0, 0;
    nanomap.SetBodyToRdf(body_to_rdf);

    pcl_sub = nh.subscribe("/flight/r200/points_xyz", 100, &NanoMapNode::PointCloudCallback, this);
    pose_updates_sub = nh.subscribe("/samros/keyposes", 100, &NanoMapNode::SmoothedPosesCallback, this);
    pose_sub = nh.subscribe("/pose", 100, &NanoMapNode::PoseCallback, this);
  };

  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Subscriber pose_updates_sub;
  ros::Subscriber pose_sub;
  NanoMap nanomap;
  NanoMapVisualizer nanomap_visualizer;

  bool initialized = false;

  void PointCloudCallback(const sensor_msgs::PointCloud2& msg) {
    PointCloud<PointXYZ>::Ptr cloud_rdf(new PointCloud<PointXYZ>);
    fromSensorMsgsPointCloud2(msg,*cloud_rdf);
    NanoMapTime nm_time(msg.header.stamp.sec, msg.header.stamp.nsec);
    nanomap.AddPointCloud(cloud_rdf, nm_time, msg.header.seq);
  }

  void DrawNanoMapVisualizer() {
    std::vector<Matrix4> edges = nanomap.GetCurrentEdges();
    nanomap_visualizer.DrawFrustums(edges);
  }

  void PoseCallback(geometry_msgs::PoseStamped const& pose) {
    Eigen::Quaterniond quat(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
    Vector3 pos = Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    NanoMapTime nm_time(pose.header.stamp.sec, pose.header.stamp.nsec);
    NanoMapPose nm_pose(pos, quat, nm_time);
    nanomap.AddPose(nm_pose);

    // todo: abstract this into SetLastPose
    Matrix4 transform = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) = nm_pose.quaternion.toRotationMatrix();
    transform.block<3,1>(0,3) = nm_pose.position;
    nanomap_visualizer.SetLastPose(transform);

    DrawNanoMapVisualizer();
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "nanomap_visualization_example");
    NanoMapNode nanomap_node;
    ros::spin();
    return 0;
}
