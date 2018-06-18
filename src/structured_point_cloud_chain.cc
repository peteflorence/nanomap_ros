#include "structured_point_cloud_chain.h"
#include "pcl.h"

#define num_nearest_neighbors 1

Vector3 EdgeVertex::ApplyEdgeTransform(Vector3 const &p,
                                       Matrix4 const &transform) const {
  Vector4 p_aug;
  p_aug << p, 1.0;
  p_aug = transform * p_aug;
  return Vector3(p_aug(0), p_aug(1), p_aug(2));
}

Vector3 EdgeVertex::ApplyEdgeRotation(Vector3 const &p,
                                      Matrix3 const &rotation) const {
  return rotation * p;
}

std::vector<Matrix4> StructuredPointCloudChain::GetCurrentEdges() const {
  std::vector<Matrix4> edges;
  int chain_size = chain.size();
  for (int i = 0; i < chain_size; i++) {
    edges.push_back(chain.at(i).edge);
  }
  return edges;
}

NanoMapTime StructuredPointCloudChain::GetMostRecentCloudTime() const {
  if (NANOMAP_DEBUG_PRINT) {
    std::cout << "GetMostRecentCloudTime" << std::endl;
  }
  return chain.at(0).vertex->GetTime();
}

NanoMapTime StructuredPointCloudChain::GetOldestCloudTime() const {
  if (NANOMAP_DEBUG_PRINT) {
    std::cout << "GetOldestCloudTime" << std::endl;
  }
  return chain.back().vertex->GetTime();
}

NanoMapTime StructuredPointCloudChain::GetCloudTimeAtIndex(size_t index) const {
  if (NANOMAP_DEBUG_PRINT) {
    std::cout << "GetCloudTimeAtIndex" << std::endl;
  }
  return chain.at(index).vertex->GetTime();
  if (NANOMAP_DEBUG_PRINT) {
    std::cout << "got it" << std::endl;
  }
}

void StructuredPointCloudChain::DeleteMemoryBeforeTime(
    NanoMapTime const &delete_time) {
  while (chain.size() >= 0) {
    NanoMapTime i = chain.back().vertex->GetTime();
    if (delete_time.GreaterThan(i)) {
      chain.pop_back();
    } else {
      break;
    }
  }
}

void StructuredPointCloudChain::SetNumDepthImageHistory(
    int N_depth_image_history) {
  N_max_point_clouds = N_depth_image_history;
}

size_t StructuredPointCloudChain::GetChainSize() const { return chain.size(); }

void StructuredPointCloudChain::SetBodyToRdf(Matrix3 const &body_to_rdf) {
  _body_to_rdf = body_to_rdf;
  _body_to_rdf_inverse = body_to_rdf.inverse();
  _body_to_rdf_4 = Eigen::Matrix4d::Identity();
  _body_to_rdf_4_inverse = Eigen::Matrix4d::Identity();
  _body_to_rdf_4.block<3, 3>(0, 0) = _body_to_rdf;
  _body_to_rdf_4_inverse.block<3, 3>(0, 0) = _body_to_rdf_inverse;
}

void StructuredPointCloudChain::UpdateEdge(uint32_t index,
                                           Matrix4 const &relative_transform) {
  chain.at(index).edge = relative_transform;
  chain.at(index).edge_rotation_only = relative_transform.block<3, 3>(0, 0);
  //  if (index = 0) {
  //    chain.at(0).edge_rdf = _body_to_rdf_4 * relative_transform;
  //    chain.at(0).edge_rdf_rotation_only = _body_to_rdf *
  //    relative_transform.block<3,3>(0,0);
  //  }
  chain.at(index).edge_rdf =
      _body_to_rdf_4 * relative_transform * _body_to_rdf_4_inverse;
  chain.at(index).edge_rdf_rotation_only =
      _body_to_rdf * relative_transform.block<3, 3>(0, 0) *
      _body_to_rdf_inverse;
}

void StructuredPointCloudChain::ManageChainSize() {
  while (chain.size() > N_max_point_clouds) {
    chain.pop_back();
  }
}

void StructuredPointCloudChain::AddNextEdgeVertex(
    Matrix4 const &new_edge, StructuredPointCloudPtr const &new_cloud) {
  EdgeVertex new_edge_vertex;
  // new_edge_vertex.edge = new_edge;
  new_edge_vertex.vertex = new_cloud;
  chain.push_front(new_edge_vertex);
  UpdateEdge(0, new_edge);
  ManageChainSize();
}

NanoMapKnnReply
StructuredPointCloudChain::KnnQuery(NanoMapKnnArgs const &args) const {
  NanoMapKnnReply reply;

  if (chain.size() == 0) {
    reply.fov_status = NanoMapFovStatus::empty_memory;
    return reply;
  }

  Vector3 search_position_rdf =
      _body_to_rdf * args.query_point_current_body_frame; // not currently in
                                                          // rdf, but will be
                                                          // after first
                                                          // transform
  // Vector3 search_position_rdf = _body_to_rdf * search_position;

  Vector3 sigma_rdf =
      _body_to_rdf *
      args.axis_aligned_linear_covariance; // also not currently but will be
  // Vector3 sigma_rdf = _body_to_rdf * sigma;

  NanoMapFovStatus first_fov_status;
  uint32_t first_frame_id;
  Vector3 first_sigma_rdf;
  Vector3 first_search_position_rdf;

  // search through chain
  for (auto i = chain.cbegin(); i != chain.cend(); ++i) {
    // transform to previous body frame
    search_position_rdf =
        i->ApplyEdgeTransform(search_position_rdf, i->edge_rdf);
    sigma_rdf = i->ApplyEdgeRotation(sigma_rdf, i->edge_rdf_rotation_only);
    // double sigma_each_direction = 0.013; // sigma increase up to 2 meters
    // over 150
    sigma_rdf = sigma_rdf + Vector3(0.1, 0.1, 0.1);

    // transform into sensor rdf frame
    // search_position_rdf =
    // i->vertex->fov_evaluator_->RotateToSensorFrame(search_position);
    // sigma_rdf           =
    // i->vertex->fov_evaluator_->RotateToSensorFrame(sigma);

    // check fov
    NanoMapFovStatus fov_status = i->vertex->fov_evaluator_->EvaluateFov(
        i->vertex->cloud_ptr_, search_position_rdf, sigma_rdf * 0.0,
        (i == chain.cbegin()));
    // NanoMapFovStatus fov_status = NanoMapFovStatus::behind;

    if (i == chain.cbegin()) {
      first_fov_status = fov_status;
      first_frame_id = i->vertex->frame_id_;
      first_sigma_rdf = sigma_rdf;
      first_search_position_rdf = search_position_rdf;
    }

    // if free, do NN and return
    if (fov_status == NanoMapFovStatus::free_space || args.early_exit) {
      i->vertex->kd_tree_.SearchForNearest<num_nearest_neighbors>(
          search_position_rdf[0], search_position_rdf[1],
          search_position_rdf[2]);
      std::vector<PointXYZ> closest_pts = i->vertex->kd_tree_.closest_pts;
      std::vector<Vector3> return_points;
      if (closest_pts.size() > 0) {
        for (size_t j = 0;
             j < std::min((int)closest_pts.size(), num_nearest_neighbors);
             j++) {
          PointXYZ next_point = closest_pts[j];
          Vector3 depth_position =
              Vector3(next_point.x, next_point.y, next_point.z);
          return_points.push_back(depth_position);
        }
      }
      reply.fov_status = fov_status;
      reply.frame_id = i->vertex->frame_id_;
      reply.query_point_in_frame_id = search_position_rdf;
      reply.closest_points_in_frame_id = return_points;
      reply.axis_aligned_linear_covariance = sigma_rdf;
      return reply;
    }
  }
  chain.at(0).vertex->kd_tree_.SearchForNearest<num_nearest_neighbors>(
      first_search_position_rdf[0], first_search_position_rdf[1],
      first_search_position_rdf[2]);

  std::vector<PointXYZ> closest_pts =
      chain.at(0).vertex->kd_tree_.closest_pts;
  if (0) {
    std::cout << "closest_pts.size() in nanomap kd_tree" << closest_pts.size()
              << std::endl;
  }
  std::vector<Vector3> return_points;
  if (closest_pts.size() > 0) {
    for (size_t j = 0;
         j < std::min((int)closest_pts.size(), num_nearest_neighbors); j++) {
      if (0) {
        std::cout << "ADDING" << std::endl;
      }
      PointXYZ next_point = closest_pts[j];
      Vector3 depth_position =
          Vector3(next_point.x, next_point.y, next_point.z);
      return_points.push_back(depth_position);
      if (0) {
        std::cout << "return_points now this big " << return_points.size()
                  << std::endl;
      }
    }
  }

  reply.fov_status = first_fov_status;
  reply.frame_id = first_frame_id;
  reply.query_point_in_frame_id = first_search_position_rdf;
  reply.closest_points_in_frame_id = return_points;
  reply.axis_aligned_linear_covariance = first_sigma_rdf;
  return reply;
}
