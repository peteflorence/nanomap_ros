#ifndef STRUCTURED_POINT_CLOUD_CHAIN_H
#define STRUCTURED_POINT_CLOUD_CHAIN_H

#include "nanomap_types.h"
#include "structured_point_cloud.h"


struct NextEdgeVertex {
	Matrix4f			 edge;
	StructuredPointCloud vertex;
};

class StructuredPointCloudChain {
  public:

  	NanoMapTime GetMostRecentCloudTime();
  	void DeleteMemoryBeforeTime(NanoMapTime delete_time);

  	void UpdateEdge(uint32_t index, Matrix4f relative_transform);
  	void AddNextEdgeVertex(Matrix4f new_edge, StructuredPointCloud new_cloud);


  private:
	std::vector<NextEdgeVertex> chain;
};

#endif