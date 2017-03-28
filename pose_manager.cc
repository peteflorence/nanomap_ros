#include "pose_manager.h"

void PoseManager::AddPose(NanoMapPose const& pose) {
	poses.push_front(pose);
}

void PoseManager::DeleteMemoryBeforeTime(NanoMapTime const& delete_time) {
	while (poses.size() >= 0) {
		NanoMapPose i = poses.back(); 
		if ( (i.time.sec <= delete_time.sec) && (i.time.nsec < delete_time.nsec) ) {
			poses.pop_back();
		}
		else {
			break;
		}
	}
}

NanoMapTime PoseManager::GetMostRecentPoseTime() const {
	return poses.front().time;
}

bool PoseManager::CanInterpolatePoseAtTime(NanoMapTime const& query_time) const {
	if (poses.size() == 0) {
		return false;
	} 

	NanoMapTime oldest_time = poses.back().time;
	NanoMapTime newest_time = poses.front().time;

	if ( (query_time.sec <= oldest_time.sec) && (query_time.nsec < oldest_time.nsec) ) {
		return false;
	}
	if ( (query_time.sec >= newest_time.sec) && (query_time.nsec > newest_time.nsec) ) {
		return false;
	}

	return true;
}

bool PoseManager::CanInterpolatePosesForTwoTimes(NanoMapTime const& time_from, NanoMapTime const& time_to) const {
	return (CanInterpolatePoseAtTime(time_from) && CanInterpolatePoseAtTime(time_to) );	
}

NanoMapPose PoseManager::GetPoseAtTime(NanoMapTime const& query_time) {
	// iterate through pose times and find bookends
	size_t oldest_pose_index = poses.size()-1;

    NanoMapPose pose_before = poses[oldest_pose_index];
    NanoMapPose pose_after;
    for (size_t i = oldest_pose_index - 1; i >= 0; i--) {
      pose_after = poses[i];
      if ((pose_after.time.sec > query_time.sec) && (pose_after.time.nsec > query_time.nsec)) {
        break;
      }
      pose_before = pose_after;
    }

    // find pose interpolation parameter
    double t_1  = (query_time.sec  - pose_before.time.sec) * 1.0 + (query_time.nsec - pose_before.time.nsec)/ 1.0e9;
    double t_2  = (pose_after.time.sec  - query_time.sec)  * 1.0 + (pose_after.time.nsec - query_time.nsec) / 1.0e9;

    double t_parameter = t_1 / (t_1 + t_2);

	// interpolate
	return InterpolateBetweenPoses(pose_before, pose_after, t_parameter);
}

NanoMapPose PoseManager::InterpolateBetweenPoses(NanoMapPose const& pose_before, NanoMapPose const& pose_after, double t_parameter) {

	// position interpolation
	Vector3 interpolated_vector = pose_before.position + (pose_after.position - pose_before.position)*t_parameter;

	// quaternion interpolation
	Quat interpolated_quat;
	interpolated_quat = pose_before.quaternion.slerp(t_parameter, pose_after.quaternion);

	NanoMapPose pose;
	pose.position = interpolated_vector;
	pose.quaternion = interpolated_quat;
	return pose;
}