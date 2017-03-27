#ifndef NANOMAP_H
#define NANOMAP_H

#include "nanomap_types.h"
#include "pose_manager.h"
#include "structured_point_cloud_chain.h"
#include "fov_evaluator.h"

class NanoMap {
 public:

  void AddPose(NanoMapPose pose);
  void AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime cloud_time);
  void DeleteMemoryBeforeTime(NanoMapTime delete_time);
  void SetCameraInfo(double bin, double width, double height, Matrix3 K_camera_info);

  NanoMapKnnReply KnnQuery(NanoMapKnnArgs);

 private:
  void UpdateChainWithLatestPose();
  void TryAddingPointCloudBufferToChain();

  PoseManager pose_manager;
  std::deque<StructuredPointCloud> point_cloud_buffer;

  StructuredPointCloudChain structured_point_cloud_chain;

};

#endif