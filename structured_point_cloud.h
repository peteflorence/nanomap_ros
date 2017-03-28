#ifndef STRUCTURED_POINT_CLOUD_H
#define STRUCTURED_POINT_CLOUD_H

#include "nanomap_types.h"
#include "kd_tree_two.h"
#include "fov_evaluator.h"


class StructuredPointCloud {
 public:
  FovEvaluatorPtr fov_evaluator_;
  PointCloudPtr cloud_ptr_;

  StructuredPointCloud(PointCloudPtr const& cloud_ptr, NanoMapTime const& cloud_time, FovEvaluatorPtr const& fov_evaluator) {
  	cloud_ptr_ = cloud_ptr;
  	kd_tree_.Initialize(cloud_ptr_, false);
  	fov_evaluator_ = fov_evaluator;
  };

  NanoMapTime GetTime() const{
  	return cloud_time_;
  };

  PointCloudPtr GetPointCloudPtr() const{
  	return cloud_ptr_;
  };


 private:
 	KDTreeTwo<double> kd_tree_;
 	NanoMapTime cloud_time_;
};

typedef std::shared_ptr<StructuredPointCloud> StructuredPointCloudPtr;

#endif