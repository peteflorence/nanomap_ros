#include "nanomap.h"

#define num_nearest_neighbors 1

void NanoMap::AddPose(NanoMapPose pose) {
  pose_manager.AddPose(pose);

  // if there are point clouds on buffer, see if can add them to chain now

  // update transform to latest point cloud
}

void NanoMap::DeleteMemoryBeforeTime(NanoMapTime time) {
  pose_manager.DeleteMemoryBeforeTime(time);
}


void NanoMap::AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime cloud_time) {
  // build structured_point_cloud
  StructuredPointCloud new_cloud = StructuredPointCloud(cloud_ptr, cloud_time);

  if (pose_manager.HavePoseAtTime(cloud_time)) {
    // add point cloud to chain
  }
  else {
    point_cloud_buffer.push_back(new_cloud);
  }


}

void NanoMap::SetCameraInfo(double bin, double width, double height, Matrix3 K_camera_info) {
  return;
}