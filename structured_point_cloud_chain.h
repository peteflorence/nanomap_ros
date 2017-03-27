#ifndef STRUCTURED_POINT_CLOUD_CHAIN_H
#define STRUCTURED_POINT_CLOUD_CHAIN_H

#include "structured_point_cloud.h"

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
typedef Eigen::Matrix<float, 4, 4>  Matrix4f; 

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