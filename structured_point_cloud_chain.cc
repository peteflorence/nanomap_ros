#include "structured_point_cloud_chain.h"

Vector3 EdgeVertex::ApplyEdgeTransform(Vector3 p) const {
  Vector4 p_aug;
  p_aug << p, 1.0;
  p_aug = edge * p_aug;
  return Vector3(p_aug(0), p_aug(1), p_aug(2));
}

void StructuredPointCloudChain::DeleteMemoryBeforeTime(NanoMapTime const& delete_time) {
	while (chain.size() >= 0) {
		NanoMapTime i = chain.at(0).vertex->GetTime(); 
		if ( (i.sec <= delete_time.sec) && (i.nsec < delete_time.nsec) ) {
			chain.pop_front();
		}
		else {
			break;
		}
	}
}

NanoMapKnnReply const StructuredPointCloudChain::KnnQuery(NanoMapKnnArgs const& args) const {
  NanoMapKnnReply reply;

  // if chain is empty, return
  if  (chain.size() == 0) {
  	reply.fov_status = NanoMapFovStatus::empty_memory;
  	return reply;
  }

  Vector3 search_position = args.query_point_current_body_frame;
  Vector3 search_position_rdf = Vector3(0,0,0);

  // search through chain
  for (auto i = chain.begin(); i != chain.end(); ++i) { 

  	// transform to previous body frame
  	search_position = i->ApplyEdgeTransform(search_position);

  	// transform into sensor rdf frame
  	search_position_rdf = i->vertex->fov_evaluator_->RotateToSensorFrame(search_position);

  	// check fov
  	NanoMapFovStatus fov_status = i->vertex->fov_evaluator_->EvaluateFov(i->vertex->cloud_ptr_, search_position_rdf);

  	// potentially do NN

  }

  reply.fov_status;
  reply.frame_id;
  reply.query_point_in_frame_id;
  reply.closest_points_in_frame_id;
  return reply;
}