#ifndef POSE_MANAGER_H
#define POSE_MANAGER_H

#include "nanomap_types.h"

class PoseManager {
public:
  void AddPose(NanoMapPose const &pose);
  void DeleteMemoryBeforeTime(NanoMapTime const &time);
  void DeleteMemoryInBetweenTime(NanoMapTime const &time_before,
                                 NanoMapTime const &time_after);

  NanoMapTime GetMostRecentPoseTime() const;
  NanoMapTime GetOldestPoseTime() const;
  bool CanInterpolatePoseAtTime(NanoMapTime const &query_time) const;
  bool CanInterpolatePosesForTwoTimes(NanoMapTime const &time_from,
                                      NanoMapTime const &time_to) const;
  NanoMapTime GetTimeOfPoseBefore(NanoMapTime const &time) const;
  NanoMapPose GetPoseAtTime(NanoMapTime const &query_time);
  Matrix4 GetRelativeTransformFromTo(NanoMapTime const &time_from,
                                     NanoMapTime const &time_to);

  size_t GetNumPoses() { return poses.size(); };

private:
  NanoMapPose InterpolateBetweenPoses(NanoMapPose const &pose_before,
                                      NanoMapPose const &pose_after,
                                      double t_parameter);
  Matrix4 FindTransform(NanoMapPose const &new_pose,
                        NanoMapPose const &previous_pose);
  Matrix4 FindTransform(NanoMapPose const &pose);
  Matrix4 InvertTransform(Matrix4 const &transform);
  void CheckMonotonic() const;
  void PrintAllPosePositions();

  std::deque<NanoMapPose> poses;
};

#endif