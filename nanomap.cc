#include "nanomap.h"

#define num_nearest_neighbors 1

void NanoMap::AddPose(NanoMapPose pose) {
  pose_manager.AddPose(pose);

  // try adding point clouds off buffer
  TryAddingPointCloudBufferToChain();

  // update last transform only
  UpdateChainWithLatestPose();
}

void NanoMap::AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime cloud_time) {
  // build structured_point_cloud
  StructuredPointCloud new_cloud = StructuredPointCloud(cloud_ptr, cloud_time);

  // add it to buffer
  point_cloud_buffer.push_back(new_cloud);

  // try adding point clouds off buffer
  TryAddingPointCloudBufferToChain();

}

void NanoMap::DeleteMemoryBeforeTime(NanoMapTime delete_time) {
  pose_manager.DeleteMemoryBeforeTime(delete_time);
  structured_point_cloud_chain.DeleteMemoryBeforeTime(delete_time);
}

void NanoMap::SetCameraInfo(double bin, double width, double height, Matrix3 K_camera_info) {
  return;
}

void NanoMap::UpdateChainWithLatestPose() {
  NanoMapTime previous_cloud_time = structured_point_cloud_chain.GetMostRecentCloudTime();
  NanoMapTime last_pose_time = pose_manager.GetMostRecentPoseTime();
  Matrix4f updated_transform = pose_manager.GetRelativeTransformFromTo(last_pose_time, previous_cloud_time);
  structured_point_cloud_chain.UpdateEdge(0, updated_transform);
}

void NanoMap::TryAddingPointCloudBufferToChain() {
  while (point_cloud_buffer.size() > 0) {

    StructuredPointCloud new_cloud = point_cloud_buffer.at(0);
    NanoMapTime new_cloud_time = new_cloud.GetTime();

    if (pose_manager.CanInterpolatePoseAtTime(new_cloud_time)) {
      NanoMapTime previous_cloud_time = structured_point_cloud_chain.GetMostRecentCloudTime();
      Matrix4f previous_edge = pose_manager.GetRelativeTransformFromTo(new_cloud_time, previous_cloud_time);
      structured_point_cloud_chain.UpdateEdge(0, previous_edge);

      NanoMapTime last_pose_time = pose_manager.GetMostRecentPoseTime();
      Matrix4f new_edge = pose_manager.GetRelativeTransformFromTo(last_pose_time, new_cloud_time);
      structured_point_cloud_chain.AddNextEdgeVertex(new_edge, new_cloud);

      point_cloud_buffer.pop_front();

    } else {
      break;
    }

  }
}