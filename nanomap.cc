#include "nanomap.h"
#include <algorithm>

NanoMap::NanoMap() : 
received_camera_info(false),
received_sensor_transform(false)
{
  fov_evaluator_ptr = std::make_shared<FovEvaluator>();
}

void NanoMap::AddPose(NanoMapPose const& pose) {
  if (NANOMAP_DEBUG_PRINT){std::cout << "In AddPose" << std::endl;}
  pose_manager.AddPose(pose);

  // try adding point clouds off buffer
  TryAddingPointCloudBufferToChain();

  // update last transform only
  UpdateChainWithLatestPose();
  if (NANOMAP_DEBUG_PRINT){std::cout << "Exiting AddPose" << std::endl;}
}

void NanoMap::AddPoseUpdates(std::vector<NanoMapPose>& pose_updates) {
  // check at least 2 poses in pose updates
  if (pose_updates.size() < 2) {
    if (NANOMAP_DEBUG_PRINT){std::cout << "Can only handle pose updates" << std::endl;}
    return;
  }

  // make it so that pose_updates are NEWEST up front, if not then reverse order
  if (pose_updates.back().time.GreaterThan(pose_updates.front().time)) {
    std::reverse(pose_updates.begin(), pose_updates.end());
  }

  // check at least 10 Hz poses
  double duration = (pose_updates.front().time.sec  - pose_updates.back().time.sec) * 1.0 + (pose_updates.front().time.nsec - pose_updates.back().time.nsec)/ 1.0e9;
  double pose_update_frequency = pose_updates.size() / duration;
  if (pose_update_frequency < 10.0) {
    if (NANOMAP_DEBUG_PRINT){std::cout << "Only safe to update poses for frequency > 10 Hz" << std::endl;}
    return;
  }

  // delete previous poses in this time frame
  pose_manager.DeleteMemoryInBetweenTime(pose_updates.back().time, pose_updates.front().time);

  // add updated poses
  size_t pose_updates_size = pose_updates.size();
  for (size_t i = 0; i < pose_updates_size; i++) {
    if (pose_manager.GetOldestPoseTime().GreaterThan(pose_updates.at(i).time)) {
      break;
    }
    pose_manager.AddPose(pose_updates.at(i));
  }
}

void NanoMap::AddPointCloud(PointCloudPtr const& cloud_ptr, NanoMapTime const& cloud_time, uint32_t frame_id) {
  if (NANOMAP_DEBUG_PRINT){std::cout << "In AddPointCloud" << std::endl;}

  // do not add if no poses at all or older poses (unlikely we will ever be able to interpolate)
  if (pose_manager.GetNumPoses() == 0) {
    return;
  }
  NanoMapTime oldest_pose_time = pose_manager.GetOldestPoseTime();
  if (oldest_pose_time.GreaterThan(cloud_time)) {
    return;
  } 

  // build structured_point_cloud and add to buffer
  StructuredPointCloudPtr new_cloud_ptr = std::make_shared<StructuredPointCloud>(cloud_ptr, cloud_time, frame_id, fov_evaluator_ptr);
  point_cloud_buffer.push_back(new_cloud_ptr);
  if (NANOMAP_DEBUG_PRINT){std::cout << "Pushed on buffer, buffer now this big: " << point_cloud_buffer.size() << std::endl;}

  // try adding point clouds off buffer to chain 
  TryAddingPointCloudBufferToChain();
  NanoMapDebugPrintState();
  TrimPoseMemory();
  if (NANOMAP_DEBUG_PRINT){std::cout << "Exiting AddPointCloud" << std::endl;}
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
    NanoMapTime previous_cloud_time = structured_point_cloud_chain.GetMostRecentCloudTime();
    NanoMapTime last_pose_time = pose_manager.GetMostRecentPoseTime();
    Matrix4 updated_transform = pose_manager.GetRelativeTransformFromTo(last_pose_time, previous_cloud_time);
    structured_point_cloud_chain.UpdateEdge(0, updated_transform);
  }
}

void NanoMap::TryAddingPointCloudBufferToChain() {
  while (point_cloud_buffer.size() > 0) {
    StructuredPointCloudPtr new_cloud_ptr = point_cloud_buffer.at(0);
    NanoMapTime new_cloud_time = new_cloud_ptr->GetTime();

    if (pose_manager.CanInterpolatePoseAtTime(new_cloud_time)) {
      if (NANOMAP_DEBUG_PRINT){std::cout << "TryAdding and can interpolate" << std::endl;}

      if (structured_point_cloud_chain.GetChainSize() > 0) {
        if (NANOMAP_DEBUG_PRINT){std::cout << "ChainSize greater than 0" << std::endl;}
        NanoMapTime previous_cloud_time = structured_point_cloud_chain.GetMostRecentCloudTime();
        if (NANOMAP_DEBUG_PRINT){std::cout << "Got previous_cloud_time " << previous_cloud_time.sec << "." << previous_cloud_time.nsec << std::endl;}
        Matrix4 previous_edge = pose_manager.GetRelativeTransformFromTo(new_cloud_time, previous_cloud_time);
        if (NANOMAP_DEBUG_PRINT){std::cout << "Got relative transform " << std::endl;}
        structured_point_cloud_chain.UpdateEdge(0, previous_edge);
        if (NANOMAP_DEBUG_PRINT){std::cout << "Updated 0 edge " << std::endl;}
      }

      if (NANOMAP_DEBUG_PRINT){std::cout << "## try to get most recent pose time" << std::endl;}
      NanoMapTime last_pose_time = pose_manager.GetMostRecentPoseTime();
      if (NANOMAP_DEBUG_PRINT){std::cout << "## try to get relative transform" << std::endl;}
      Matrix4 new_edge = pose_manager.GetRelativeTransformFromTo(last_pose_time, new_cloud_time);
      if (NANOMAP_DEBUG_PRINT){std::cout << "## new_edge " << new_edge << std::endl;} 
      if (NANOMAP_DEBUG_PRINT){std::cout << "## try to add edgevertex" << std::endl;}
      structured_point_cloud_chain.AddNextEdgeVertex(new_edge, new_cloud_ptr);
      if (NANOMAP_DEBUG_PRINT){std::cout << "try to pop front of point_cloud_buffer" << std::endl;}

      point_cloud_buffer.pop_front();

    } else {
      if (NANOMAP_DEBUG_PRINT){std::cout << "breaking out of TryAdding" << std::endl;}
      break;
    }

  }
}

void NanoMap::TrimPoseMemory() {
  if (structured_point_cloud_chain.GetChainSize() > 0) {
    NanoMapTime oldest_cloud_time = structured_point_cloud_chain.GetOldestCloudTime();
    NanoMapTime time_of_pose_before = pose_manager.GetTimeOfPoseBefore(oldest_cloud_time);
    pose_manager.DeleteMemoryBeforeTime(time_of_pose_before);
  }
}

NanoMapKnnReply NanoMap::KnnQuery(NanoMapKnnArgs const& args) const {
  //std::cout << "Entering KnnQuery" << std::endl;
  if (received_camera_info && received_sensor_transform) {
    //std::cout << "Calling down to structured_point_cloud_chain" << std::endl;
    return structured_point_cloud_chain.KnnQuery(args);    
  }
  else {
    NanoMapKnnReply reply;
    reply.fov_status = NanoMapFovStatus::not_initialized;
    return reply;
  }

}

void NanoMap::NanoMapDebugPrintState() {
  if (NANOMAP_DEBUG_PRINT){
  std::cout << std::endl;
  std::cout << "received_sensor_transform" << received_sensor_transform << std::endl;
  std::cout << "received_camera_info     " << received_camera_info << std::endl;
  std::cout << "point_cloud_buffer.size() " << point_cloud_buffer.size() << std::endl;
  std::cout << "poses.size()"               << pose_manager.GetNumPoses() << std::endl;
  std::cout << "chain.size()"               << structured_point_cloud_chain.GetChainSize() << std::endl;
  if (structured_point_cloud_chain.GetChainSize()>0) {
    std::cout << "time of last point cloud  " << structured_point_cloud_chain.GetMostRecentCloudTime().sec << "." << structured_point_cloud_chain.GetMostRecentCloudTime().nsec << std::endl;
  }
  if (pose_manager.GetNumPoses() > 0) {
    std::cout << "time of last pose         " << pose_manager.GetMostRecentPoseTime().sec << "." << pose_manager.GetMostRecentPoseTime().nsec << std::endl;            
  }    
  std::cout << std::endl;
  }
}


