#include "nanomap.h"

#define num_nearest_neighbors 1

void NanoMap::AddPose(NanoMapPose pose) {
  pose_manager.AddPose(pose);

  // if there are point clouds on buffer, see if can add them to chain now


}

void NanoMap::DeleteMemoryBeforeTime(NanoMapTime time) {
  pose_manager.DeleteMemoryBeforeTime(time);
}


void NanoMap::AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime time) {
  // if have recent enough pose, add point clouds to chain

  // if don't, leave it in buffer

}

void NanoMap::SetCameraInfo(double bin, double width, double height, Matrix3 K_camera_info) {
  return;
}