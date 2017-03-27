#ifndef STRUCTURED_POINT_CLOUD_H
#define STRUCTURED_POINT_CLOUD_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
typedef Eigen::Matrix<float, 4, 4>  Matrix4f; 

class StructuredPointCloud {
 public:

  StructuredPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr) {

  };

 private:
 	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;

};

#endif