#ifndef FOV_EVALUATOR_H
#define FOV_EVALUATOR_H

#include "nanomap_types.h"
#include "structured_point_cloud.h"

class FovEvaluator {
 public:
   NanoMapFovStatus EvaluateFOV(StructuredPointCloudPtr point_cloud_ptr, Vector3 position);

 private:

  bool IsBehind(Vector3 position);
  bool IsOutsideDeadBand(Vector3 position);
  double IsOutsideFOV(Vector3 position);

};

#endif