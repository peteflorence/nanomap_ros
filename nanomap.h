#ifndef NANOMAP_H
#define NANOMAP_H

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "pose_manager.h"
#include "structured_point_cloud.h"
#include "structured_point_cloud_chain.h"
#include "fov_evaluator.h"

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

class NanoMap {
 public:

  void AddPose(NanoMapPose pose) {
    pose_manager.AddPose(pose);
  };
  void DeleteMemoryBeforeTime(NanoMapTime time) {
    pose_manager.DeleteMemoryBeforeTime(time);
  };

  void AddPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& xyz_cloud_ptr, NanoMapTime time){};
  void SetCameraInfo(double bin, double width, double height, Matrix3 K_camera_info);

  NanoMapKnnReply KnnQuery(NanoMapKnnArgs);

 private:
  PoseManager pose_manager;

};

#endif