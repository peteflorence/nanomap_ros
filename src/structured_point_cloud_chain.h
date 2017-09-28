#ifndef STRUCTURED_POINT_CLOUD_CHAIN_H
#define STRUCTURED_POINT_CLOUD_CHAIN_H

#include "nanomap_types.h"
#include "structured_point_cloud.h"

struct EdgeVertex {
  Vector3 ApplyEdgeTransform(Vector3 const &p, Matrix4 const &transform) const;
  Vector3 ApplyEdgeRotation(Vector3 const &p, Matrix3 const &rotation) const;
  Matrix4 edge_rdf;
  Matrix3 edge_rdf_rotation_only;
  Matrix4 edge;
  Matrix3 edge_rotation_only;
  StructuredPointCloudPtr vertex;
};

class StructuredPointCloudChain {
public:
  NanoMapTime GetMostRecentCloudTime() const;
  NanoMapTime GetOldestCloudTime() const;
  NanoMapTime GetCloudTimeAtIndex(size_t index) const;
  void DeleteMemoryBeforeTime(NanoMapTime const &delete_time);

  void SetNumDepthImageHistory(int N_depth_image_history);
  size_t GetChainSize() const;
  void SetBodyToRdf(Matrix3 const &body_to_rdf);
  void UpdateEdge(uint32_t index, Matrix4 const &relative_transform);
  void AddNextEdgeVertex(Matrix4 const &new_edge,
                         StructuredPointCloudPtr const &new_cloud);

  NanoMapKnnReply KnnQuery(NanoMapKnnArgs const &args) const;

  std::vector<Matrix4> GetCurrentEdges() const;

private:
  void ManageChainSize();
  Matrix3 _body_to_rdf;
  Matrix3 _body_to_rdf_inverse;
  Matrix4 _body_to_rdf_4;
  Matrix4 _body_to_rdf_4_inverse;

  std::deque<EdgeVertex> chain;
  int N_max_point_clouds = 1; // default to 1, set to overwrite
};

#endif
