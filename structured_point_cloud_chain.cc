#include "structured_point_cloud_chain.h"

#define num_nearest_neighbors 1

Vector3 EdgeVertex::ApplyEdgeTransform(Vector3 p) const {
  Vector4 p_aug;
  p_aug << p, 1.0;
  p_aug = edge * p_aug;
  return Vector3(p_aug(0), p_aug(1), p_aug(2));
}

NanoMapTime StructuredPointCloudChain::GetMostRecentCloudTime() const {
  return chain.at(0).vertex->GetTime();
}

void StructuredPointCloudChain::DeleteMemoryBeforeTime(NanoMapTime const& delete_time) {
	while (chain.size() >= 0) {
		NanoMapTime i = chain.back().vertex->GetTime(); 
		if ( (i.sec <= delete_time.sec) && (i.nsec < delete_time.nsec) ) {
			chain.pop_back();
		}
		else {
			break;
		}
	}
}

size_t StructuredPointCloudChain::GetChainSize() const {
  return chain.size();
}

void StructuredPointCloudChain::UpdateEdge(uint32_t index, Matrix4 const& relative_transform) {
	chain.at(index).edge = relative_transform;
}

void StructuredPointCloudChain::ManageChainSize() {
  while (chain.size() > N_max_point_clouds) {
    chain.pop_back();
  }
}

void StructuredPointCloudChain::AddNextEdgeVertex(Matrix4 const& new_edge, StructuredPointCloudPtr const& new_cloud) {
	EdgeVertex new_edge_vertex;
	new_edge_vertex.edge = new_edge;
	new_edge_vertex.vertex = new_cloud;
	chain.push_front(new_edge_vertex);
  ManageChainSize();
}

NanoMapKnnReply StructuredPointCloudChain::KnnQuery(NanoMapKnnArgs const& args) const {
  NanoMapKnnReply reply;

  if  (chain.size() == 0) {
  	reply.fov_status = NanoMapFovStatus::empty_memory;
  	return reply;
  }

  Vector3 search_position = args.query_point_current_body_frame;
  Vector3 search_position_rdf = Vector3(0,0,0);
  NanoMapFovStatus first_fov_status;
  uint32_t first_frame_id;

  // search through chain
  for (auto i = chain.cbegin(); i != chain.cend(); ++i) { 

  	// transform to previous body frame
  	search_position = i->ApplyEdgeTransform(search_position);

  	// transform into sensor rdf frame
  	search_position_rdf = i->vertex->fov_evaluator_->RotateToSensorFrame(search_position);

  	// check fov
  	NanoMapFovStatus fov_status = i->vertex->fov_evaluator_->EvaluateFov(i->vertex->cloud_ptr_, search_position_rdf);
    // switch to this
    // NanoMapFovStatus fov_status = i->EvaluateFov(search_position_rdf);
  	if (i == chain.cbegin()) {
  		first_fov_status = fov_status;
  		first_frame_id = i->vertex->frame_id;
      // first_frame_id = i->GetFrameId();
  	}

  	// if free, do NN and break
  	if (fov_status == NanoMapFovStatus::free_space) {

  		i->vertex->kd_tree_.SearchForNearest<num_nearest_neighbors>(search_position_rdf[0], search_position_rdf[1], search_position_rdf[2]);

  		std::vector<pcl::PointXYZ> closest_pts = i->vertex->kd_tree_.closest_pts;
  		std::vector<Vector3> return_points;
  		if (closest_pts.size() > 0) {
    		for (size_t i = 0; i < std::min((int)closest_pts.size(), num_nearest_neighbors); i++) {
  				pcl::PointXYZ next_point = closest_pts[i];
     			Vector3 depth_position = Vector3(next_point.x, next_point.y, next_point.z);
     			return_points.push_back(depth_position);
     		}
     	}

     	reply.fov_status = fov_status;
     	reply.frame_id = i->vertex->frame_id;
      //reply.frame_id = i->GetVertex()->GetFrameId();
     	reply.query_point_in_frame_id = search_position;
     	reply.closest_points_in_frame_id = return_points;
     	break;
 	  }

  }

  reply.fov_status = first_fov_status;
  reply.frame_id = first_frame_id;
  reply.query_point_in_frame_id = args.query_point_current_body_frame;
  reply.closest_points_in_frame_id = std::vector<Vector3>();
  return reply;
}