#ifndef NANOMAP_H
#define NANOMAP_H

#include "fov_evaluator.h"
#include "nanomap_types.h"
#include "pose_manager.h"
#include "structured_point_cloud_chain.h"

class NanoMap {
public:
  NanoMap();

  void AddPose(NanoMapPose const &pose);
  void AddPoseUpdates(std::vector<NanoMapPose> &pose_updates);
  void AddPointCloud(PointCloudPtr const &cloud_ptr,
                     NanoMapTime const &cloud_time, uint32_t frame_id);

  void DeleteMemoryBeforeTime(NanoMapTime const &delete_time);

  void SetNumDepthImageHistory(int N_depth_image_history);
  void SetSensorRange(double range);
  void SetCameraInfo(double bin, double width, double height,
                     Matrix3 const &K_camera_info);
  void SetBodyToRdf(Matrix3 const &R_body_to_rdf);

  std::vector<Matrix4> GetCurrentEdges() const;

  NanoMapKnnReply KnnQuery(NanoMapKnnArgs const &args) const;

private:
  void UpdateChainWithLatestPose();
  void UpdateChainInBetweenTimes(NanoMapTime const &time_before,
                                 NanoMapTime const &time_after);
  void TryAddingPointCloudBufferToChain();
  void TrimPoseMemory();

  PoseManager pose_manager;
  FovEvaluatorPtr fov_evaluator_ptr;

  std::deque<StructuredPointCloudPtr> point_cloud_buffer;
  StructuredPointCloudChain structured_point_cloud_chain;

  bool received_camera_info;
  bool received_sensor_transform;

  void NanoMapDebugPrintState();
};

#endif