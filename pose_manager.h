#ifndef POSE_MANAGER_H
#define POSE_MANAGER_H

#include "nanomap_types.h"

class PoseManager {
 public:

  void AddPose(NanoMapPose const& pose);
  void DeleteMemoryBeforeTime(NanoMapTime const& time);

  NanoMapTime GetMostRecentPoseTime() const;
  bool CanInterpolatePoseAtTime(NanoMapTime const& query_time) const;
  bool CanInterpolatePosesForTwoTimes(NanoMapTime const& time_from, NanoMapTime const& time_to) const;
  NanoMapPose GetPoseAtTime(NanoMapTime const& query_time);
  Matrix4 GetRelativeTransformFromTo(NanoMapTime const& time_from, NanoMapTime const& time_to);

 private:
  std::deque<NanoMapPose> poses;

};

#endif