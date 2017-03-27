#ifndef FOV_EVALUATOR_H
#define FOV_EVALUATOR_H

#include "nanomap_types.h"

class FovEvaluator {
 public:

 private:

  bool IsBehind(Vector3 robot_position);
  bool IsOutsideDeadBand(Vector3 robot_position);
  double IsOutsideFOV(Vector3 robot_position);
  double AddOutsideFOVPenalty(Vector3 robot_position, double probability_of_collision);

};

#endif