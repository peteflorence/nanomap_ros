#ifndef STRUCTURED_POINT_CLOUD_H
#define STRUCTURED_POINT_CLOUD_H

#include "nanomap_types.h"
#include "kd_tree_two.h"


class StructuredPointCloud {
 public:

  StructuredPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime cloud_time) {
  	_cloud_ptr = cloud_ptr;
  	_kd_tree.Initialize(_cloud_ptr, false);
  };

  NanoMapTime GetTime() {
  	return _cloud_time;
  };

 private:
 	KDTreeTwo<double> _kd_tree;
 	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;
 	NanoMapTime _cloud_time;
};

typedef std::shared_ptr<StructuredPointCloud> StructuredPointCloudPtr;

#endif