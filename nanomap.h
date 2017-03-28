#ifndef NANOMAP_H
#define NANOMAP_H

#include "nanomap_types.h"
#include "pose_manager.h"
#include "structured_point_cloud_chain.h"
#include "fov_evaluator.h"

class NanoMap {
 public:

  void AddPose(NanoMapPose const& pose);
  void AddPointCloud(PointCloudPtr const& cloud_ptr, NanoMapTime const& cloud_time);
  
  void DeleteMemoryBeforeTime(NanoMapTime const& delete_time);

  void SetCameraInfo(double bin, double width, double height, Matrix3 const& K_camera_info);
  void SetBodyToRdf(Matrix3 const& R_body_to_rdf);

  NanoMapKnnReply KnnQuery(NanoMapKnnArgs const& args) const;

 private:
  void UpdateChainWithLatestPose();
  void TryAddingPointCloudBufferToChain();

  PoseManager pose_manager;
  FovEvaluatorPtr fov_evaluator_ptr;

  std::deque<StructuredPointCloudPtr> point_cloud_buffer;
  StructuredPointCloudChain structured_point_cloud_chain;

};

#endif