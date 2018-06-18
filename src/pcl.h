#ifndef POINT_XYZ_HPP
#define POINT_XYZ_HPP

#include <Eigen/Core>
#include <vector>
#include <memory>

#include <sensor_msgs/PointCloud2.h>

struct EIGEN_ALIGN16 PointXYZ {

  PointXYZ() {
    x = y = z = 0.0f;
    data[3] = 1.0f;
  }

  PointXYZ(float _x, float _y, float _z) {
    x = _x; y = _y; z = _z;
    data[3] = 1.0f;
  }

  union EIGEN_ALIGN16 {
    float data[4];
    struct {
      float x;
      float y;
      float z;
    };
  };

  friend std::ostream &operator<<(std::ostream& os, const PointXYZ& p);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename PointType>
class PointCloud {
public:
  typedef std::shared_ptr<PointCloud<PointType>> Ptr;
  std::vector<PointType> points;

  const PointType &at(int column, int row) const {
    return points.at(row * width + column);
  }

  PointType &at(int column, int row) {
    return points.at(row * width + column);
  }

  Ptr makeShared() {
    return Ptr (new PointCloud<PointType> (*this));
  }

  uint32_t width, height;
};

// TODO: This may be incorrect - assumes that the incoming message is PointXYZ and is contiguous in memory
void fromSensorMsgsPointCloud2(const sensor_msgs::PointCloud2 & pc2_msg, PointCloud<PointXYZ> &cloud);

#endif //POINT_XYZ_HPP
