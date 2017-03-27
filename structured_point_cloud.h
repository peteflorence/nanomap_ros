#ifndef STRUCTURED_POINT_CLOUD_H
#define STRUCTURED_POINT_CLOUD_H

#include "nanomap_types.h"

class StructuredPointCloud {
 public:

  StructuredPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud_ptr, NanoMapTime cloud_time) {

  };

  NanoMapTime GetTime() {
  	return _cloud_time;
  };

 private:
 	pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;
 	NanoMapTime _cloud_time;
};

#endif