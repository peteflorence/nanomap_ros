#ifndef STRUCTURED_POINT_CLOUD_CHAIN_H
#define STRUCTURED_POINT_CLOUD_CHAIN_H

#include "nanomap_types.h"
#include "structured_point_cloud.h"


struct EdgeVertex {	
    Vector3 ApplyEdgeTransform(Vector3 p) const;
	Matrix4				 	edge;
	StructuredPointCloudPtr vertex;
};

class StructuredPointCloudChain {
  public:

  	NanoMapTime GetMostRecentCloudTime() const;
  	void DeleteMemoryBeforeTime(NanoMapTime const& delete_time);

  	void UpdateEdge(uint32_t index, Matrix4 const& relative_transform);
  	void AddNextEdgeVertex(Matrix4 const& new_edge, StructuredPointCloudPtr const& new_cloud);

  	NanoMapKnnReply KnnQuery(NanoMapKnnArgs const& args) const;

  private:
	std::deque<EdgeVertex> chain;
};

#endif