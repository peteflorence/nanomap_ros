#ifndef POSE_MANAGER_H
#define POSE_MANAGER_H

#include "nanomap_types.h"

class PoseManager {
 public:

  void AddPose(NanoMapPose pose);
  void DeleteMemoryBeforeTime(NanoMapTime time);

  NanoMapTime GetMostRecentPoseTime();
  bool CanInterpolatePoseAtTime(NanoMapTime query_time);
  bool CanInterpolatePosesForTwoTimes(NanoMapTime time_from, NanoMapTime time_to);
  NanoMapPose GetPoseAtTime(NanoMapTime query_time);
  Matrix4f GetRelativeTransformFromTo(NanoMapTime time_from, NanoMapTime time_to);

 private:
  std::deque<NanoMapPose> poses;

};

#endif