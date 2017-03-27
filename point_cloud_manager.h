  while (point_cloud_buffer.size() > 0) {
    StructuredPointCloud last_cloud = point_cloud_buffer.at(0);
    if (pose_manager.HavePoseAtTime( last_cloud.GetTime() ) {

      NanoMapTime previous_cloud_time = 
      NanoMapTime latest_cloud_time = 

      pose_manager.GetRelativeTransform(time_1, time_2);
      structured_point_cloud_chain.AddNextEdgeVertex();
    } else {
      break;
    }
  }