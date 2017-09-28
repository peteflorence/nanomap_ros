#ifndef FOV_EVALUATOR_H
#define FOV_EVALUATOR_H

#include "nanomap_types.h"

class FovEvaluator {
public:
  Vector3 RotateToSensorFrame(Vector3 position_body_frame);
  NanoMapFovStatus EvaluateFov(PointCloudPtr const &cloud_ptr, Vector3 position,
                               Vector3 aabb, bool ignore_horizon) const;
  void SetCameraInfo(double bin, double width, double height,
                     Matrix3 const &K_camera_info);
  void SetSensorRange(double range);
  void SetBodyToRdf(Matrix3 const &R_body_to_rdf);

private:
  bool IsBehind(Vector3 position) const;
  bool IsOutsideDeadBand(Vector3 position) const;
  bool IsBeyondSensorHorizon(Vector3 position) const;

  Matrix3 R_body_to_rdf_;
  double sensor_range_ = 10.0;

  Matrix3 K;
  double binning = 4.0;
  double num_x_pixels = 320 / binning;
  double num_y_pixels = 240 / binning;
};

typedef std::shared_ptr<FovEvaluator> FovEvaluatorPtr;

#endif