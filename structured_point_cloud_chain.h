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

 private:
 	std::vector<NextEdgeVertex> chain;
};

#endif