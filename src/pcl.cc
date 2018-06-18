#include "pcl.h"

std::ostream &operator<<(std::ostream &os, const PointXYZ &p) {
  os << "x: " << p.x << "y: " << p.y << "z: " << p.z;
  return os;
}

void fromSensorMsgsPointCloud2(const sensor_msgs::PointCloud2 &pc2_msg, PointCloud<PointXYZ> &cloud) {
  cloud.width = pc2_msg.width;
  cloud.height = pc2_msg.height;
  uint32_t num_points = pc2_msg.width * pc2_msg.height;

  cloud.points.resize(num_points);
  uint8_t* cloud_data = reinterpret_cast<uint8_t*>(&cloud.points[0]);
  const uint8_t* msg_data = &pc2_msg.data[0];
  // copy pc2_msg.data to the point cloud data - not necessarily correct
  memcpy (cloud_data, msg_data, pc2_msg.data.size ());
}
