#include "pose_manager.h"

void PoseManager::AddPose(NanoMapPose pose) {
	poses.push_back(pose);
}

void PoseManager::DeleteMemoryBeforeTime(NanoMapTime delete_time) {
	while (poses.size() >= 0) {
		NanoMapPose i = poses.at(0); 
		if ( (i.time.sec < delete_time.sec) && (i.time.nsec < delete_time.nsec) ) {
			poses.pop_front();
		}
		else {
			break;
		}
	}
}