#ifndef STRUCTURED_POINT_CLOUD_H
#define STRUCTURED_POINT_CLOUD_H

#include "nanomap_types.h"
#include "kd_tree_two.h"
#include "fov_evaluator.h"


class StructuredPointCloud {
 public:

  StructuredPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime cloud_time, FovEvaluatorPtr fov_evaluator) {
  	cloud_ptr_ = cloud_ptr;
  	kd_tree_.Initialize(cloud_ptr_, false);
  	fov_evaluator_ = fov_evaluator;
  };

  NanoMapTime GetTime() {
  	return cloud_time_;
  };

  PointCloudPtr GetPointCloudPtr(){
  	return cloud_ptr_;
  };

 private:
 	FovEvaluatorPtr fov_evaluator_;
 	KDTreeTwo<double> kd_tree_;
 	PointCloudPtr cloud_ptr_;
 	NanoMapTime cloud_time_;
};

typedef std::shared_ptr<StructuredPointCloud> StructuredPointCloudPtr;

#endif