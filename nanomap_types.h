#ifndef NANOMAP_TYPES_H
#define NANOMAP_TYPES_H

#include <iostream>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
typedef Eigen::Matrix<float, 4, 4>  Matrix4f;

struct NanoMapKnnArgs {
  Vector3               query_point_body_frame;
};

struct NanoMapKnnReply {
  uint8_t               fov_status;
  uint32_t              frame_id;
  Vector3               query_point_in_frame_id;
  std::vector<Vector3>  closest_points_in_frame_id;
};

struct NanoMapTime {
  uint32_t sec;
  uint32_t nsec;
};

struct NanoMapPose {
  Matrix4f pose;
  NanoMapTime time;
};

#endif