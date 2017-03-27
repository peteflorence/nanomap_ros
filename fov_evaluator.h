#ifndef FOV_EVALUATOR_H
#define FOV_EVALUATOR_H

#include "nanomap_types.h"

class FovEvaluator {
 public:
   NanoMapFovStatus EvaluateFov(PointCloudPtr, Vector3 position);
   void setCameraInfo(double bin, double width, double height, Matrix3 K_camera_info);

 private:

  bool IsBehind(Vector3 position);
  bool IsOutsideDeadBand(Vector3 position);
  bool IsBeyondSensorHorizon(Vector3 position);

  Matrix3 K;
  double binning = 4.0;
  double num_x_pixels = 320/binning;
  double num_y_pixels = 240/binning;

};

typedef std::shared_ptr<FovEvaluator> FovEvaluatorPtr;

#endif