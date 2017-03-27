#ifndef POSE_MANAGER_H
#define POSE_MANAGER_H

#include "nanomap_types.h"

class PoseManager {
 public:

  void AddPose(NanoMapPose pose){};
  void DeleteMemoryBeforeTime(NanoMapTime time){};

  bool HavePoseAtTime(NanoMapTime query_time);
  NanoMapPose GetPoseAtTime(NanoMapTime query_time);
  NanoMapPose GetRelativeTransform(NanoMapTime time_from, NanoMapTime time_to);

 private:
  std::vector<NanoMapPose> poses;

};

#endif