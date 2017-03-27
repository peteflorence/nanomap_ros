#ifndef STRUCTURED_POINT_CLOUD_CHAIN_H
#define STRUCTURED_POINT_CLOUD_CHAIN_H

#include "nanomap_types.h"
#include "structured_point_cloud.h"


struct EdgeVertex {
	Matrix4f			 	edge;
	StructuredPointCloudPtr vertex;
};

class StructuredPointCloudChain {
  public:

  	NanoMapTime GetMostRecentCloudTime() const;
  	void DeleteMemoryBeforeTime(NanoMapTime const& delete_time);

  	void UpdateEdge(uint32_t index, Matrix4f const& relative_transform);
  	void AddNextEdgeVertex(Matrix4f const& new_edge, StructuredPointCloudPtr const& new_cloud);

  private:
	std::deque<EdgeVertex> chain;
};

#endif