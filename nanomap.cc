#include "nanomap.h"

NanoMap::NanoMap() : 
received_camera_info(false),
received_sensor_transform(false)
{
  fov_evaluator_ptr = std::make_shared<FovEvaluator>();
}

void NanoMap::AddPose(NanoMapPose const& pose) {
  pose_manager.AddPose(pose);

  // try adding point clouds off buffer
  TryAddingPointCloudBufferToChain();

  // update last transform only
  UpdateChainWithLatestPose();
}

void NanoMap::AddPointCloud(PointCloudPtr const& cloud_ptr, NanoMapTime const& cloud_time) {
  // build structured_point_cloud and add to buffer
  StructuredPointCloudPtr new_cloud_ptr = std::make_shared<StructuredPointCloud>(cloud_ptr, cloud_time, fov_evaluator_ptr);
  point_cloud_buffer.push_back(new_cloud_ptr);

  // try adding point clouds off buffer to chain
  TryAddingPointCloudBufferToChain();
}

void NanoMap::DeleteMemoryBeforeTime(NanoMapTime const& delete_time) {
  pose_manager.DeleteMemoryBeforeTime(delete_time);
  structured_point_cloud_chain.DeleteMemoryBeforeTime(delete_time);
}

void NanoMap::SetCameraInfo(double bin, double width, double height, Matrix3 const& K_camera_info) {
  fov_evaluator_ptr->SetCameraInfo(bin, width, height, K_camera_info);
  received_camera_info = true;
}

void NanoMap::SetBodyToRdf(Matrix3 const& R_body_to_rdf) {
  fov_evaluator_ptr->SetBodyToRdf(R_body_to_rdf);
  received_sensor_transform = true;
}

void NanoMap::UpdateChainWithLatestPose() {
  if (structured_point_cloud_chain.GetChainSize() > 0) {
    std::cout << "Or here" << std::endl;
    NanoMapTime previous_cloud_time = structured_point_cloud_chain.GetMostRecentCloudTime();
    NanoMapTime last_pose_time = pose_manager.GetMostRecentPoseTime();
    Matrix4 updated_transform = pose_manager.GetRelativeTransformFromTo(last_pose_time, previous_cloud_time);
    structured_point_cloud_chain.UpdateEdge(0, updated_transform);
  }
}

void NanoMap::TryAddingPointCloudBufferToChain() {
  while (point_cloud_buffer.size() > 0) {
    std::cout << "Shouldnt be in here" << std::endl;

    StructuredPointCloudPtr new_cloud_ptr = point_cloud_buffer.at(0);
    NanoMapTime new_cloud_time = new_cloud_ptr->GetTime();

    if (pose_manager.CanInterpolatePoseAtTime(new_cloud_time)) {
      NanoMapTime previous_cloud_time = structured_point_cloud_chain.GetMostRecentCloudTime();
      Matrix4 previous_edge = pose_manager.GetRelativeTransformFromTo(new_cloud_time, previous_cloud_time);
      structured_point_cloud_chain.UpdateEdge(0, previous_edge);

      NanoMapTime last_pose_time = pose_manager.GetMostRecentPoseTime();
      Matrix4 new_edge = pose_manager.GetRelativeTransformFromTo(last_pose_time, new_cloud_time);
      structured_point_cloud_chain.AddNextEdgeVertex(new_edge, new_cloud_ptr);

      point_cloud_buffer.pop_front();

    } else {
      break;
    }

  }
}

NanoMapKnnReply NanoMap::KnnQuery(NanoMapKnnArgs const& args) const {
  if (received_camera_info && received_sensor_transform) {
    return structured_point_cloud_chain.KnnQuery(args);    
  }
  else {
    NanoMapKnnReply reply;
    reply.fov_status = NanoMapFovStatus::not_initialized;
    return reply;
  }

}


