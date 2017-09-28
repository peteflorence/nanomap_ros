#ifndef STRUCTURED_POINT_CLOUD_H
#define STRUCTURED_POINT_CLOUD_H

#include "fov_evaluator.h"
#include "kd_tree_two.h"
#include "nanomap_types.h"

class StructuredPointCloud {
public:
  FovEvaluatorPtr fov_evaluator_;
  PointCloudPtr cloud_ptr_;
  KDTreeTwo<double> kd_tree_;
  uint32_t frame_id_;

  StructuredPointCloud(PointCloudPtr const &cloud_ptr,
                       NanoMapTime const &cloud_time, uint32_t frame_id,
                       FovEvaluatorPtr const &fov_evaluator) {
    cloud_ptr_ = cloud_ptr;
    kd_tree_.Initialize(cloud_ptr_, false);
    fov_evaluator_ = fov_evaluator;
    cloud_time_ = cloud_time;
    frame_id_ = frame_id;
  };

  NanoMapTime GetTime() const { return cloud_time_; };

  PointCloudPtr GetPointCloudPtr() const { return cloud_ptr_; };

private:
  NanoMapTime cloud_time_;
};

typedef std::shared_ptr<StructuredPointCloud> StructuredPointCloudPtr;

#endif