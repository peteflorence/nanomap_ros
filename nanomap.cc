#include "nanomap.h"

#define num_nearest_neighbors 1

void NanoMap::AddPose(NanoMapPose pose) {
  pose_manager.AddPose(pose);

  // if there are point clouds on buffer, see if can add them to chain now
  TryAddingPointCloudBufferToChain();

  // update transform to latest point cloud
}

void NanoMap::DeleteMemoryBeforeTime(NanoMapTime time) {
  pose_manager.DeleteMemoryBeforeTime(time);
}


void NanoMap::AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime cloud_time) {
  // build structured_point_cloud
  StructuredPointCloud new_cloud = StructuredPointCloud(cloud_ptr, cloud_time);

  // add it to buffer
  point_cloud_buffer.push_back(new_cloud);

  // try adding
  TryAddingPointCloudBufferToChain();

}

void NanoMap::SetCameraInfo(double bin, double width, double height, Matrix3 K_camera_info) {
  return;
}







void NanoMap::TryAddingPointCloudBufferToChain() {
  while (point_cloud_buffer.size() > 0) {

    StructuredPointCloud last_cloud = point_cloud_buffer.at(0);
    NanoMapTime last_cloud_time = last_cloud.GetTime();

    if (pose_manager.CanInterpolatePoseAtTime(last_cloud_time)) {
      NanoMapTime previous_cloud_time = structured_point_cloud_chain.GetMostRecentCloudTime();
      Matrix4f previous_edge = pose_manager.GetRelativeTransformFromTo(last_cloud_time, previous_cloud_time);
      structured_point_cloud_chain.UpdateEdge(0, previous_edge);

      NanoMapTime last_pose_time = pose_manager.GetMostRecentPoseTime();
      Matrix4f new_edge = pose_manager.GetRelativeTransformFromTo(last_pose_time, last_cloud_time);
      structured_point_cloud_chain.AddNextEdgeVertex(new_edge, last_cloud);

      point_cloud_buffer.pop_front();

    } else {
      break;
    }

  }
}