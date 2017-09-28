#ifndef NANOMAP_TYPES_H
#define NANOMAP_TYPES_H

#include <iostream>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;
typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;
typedef Eigen::Quaterniond Quat;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;

#define NANOMAP_DEBUG_PRINT false

struct NanoMapKnnArgs {
  Vector3 query_point_current_body_frame;
  Vector3 axis_aligned_linear_covariance;
  bool early_exit;
};

enum class NanoMapFovStatus {
  not_initialized,
  empty_memory,
  behind,
  laterally_outside_fov,
  beyond_sensor_horizon,
  occluded,
  free_space
};

template <typename T>
std::ostream &operator<<(
    typename std::enable_if<std::is_enum<T>::value, std::ostream>::type &stream,
    const T &e) {
  switch (e) {
  case T::not_initialized:
    return stream << "not_initialized";
  case T::empty_memory:
    return stream << "empty_memory";
  case T::behind:
    return stream << "behind";
  case T::laterally_outside_fov:
    return stream << "laterally_outside_fov";
  case T::beyond_sensor_horizon:
    return stream << "beyond_sensor_horizon";
  case T::occluded:
    return stream << "occluded";
  case T::free_space:
    return stream << "free_space";
  default:
    return stream << "[NANOMAPFOVSTATUS ENUM CLASS UNDEFINED]";
  }
}

struct NanoMapKnnReply {
  NanoMapFovStatus fov_status;
  uint32_t frame_id;
  Vector3 query_point_in_frame_id;
  std::vector<Vector3> closest_points_in_frame_id;
  Vector3 axis_aligned_linear_covariance;
};

struct NanoMapTime {
  NanoMapTime(){};
  NanoMapTime(uint32_t set_sec, uint32_t set_nsec) {
    sec = set_sec;
    nsec = set_nsec;
  };
  uint32_t sec;
  uint32_t nsec;

  bool GreaterThan(NanoMapTime time_2) const {
    if (sec > time_2.sec) {
      return true;
    }
    if ((sec >= time_2.sec) && (nsec > time_2.nsec)) {
      return true;
    }
    return false;
  }

  bool SameAs(NanoMapTime time_2) const {
    if ((sec == time_2.sec) && (nsec == time_2.nsec)) {
      return true;
    }
    return false;
  }
};

struct NanoMapPose {
  NanoMapPose(){};
  NanoMapPose(Vector3 set_position, Quat set_quaternion, NanoMapTime set_time) {
    position = set_position;
    quaternion = set_quaternion;
    time = set_time;
  }
  Vector3 position;
  Quat quaternion;
  NanoMapTime time;
  Vector3 incremental_transform_uncertainty;
};

#endif