#ifndef NANOMAP_VISUALIZER_H
#define NANOMAP_VISUALIZER_H

#include "nanomap_types.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class NanoMapVisualizer {
 public:

 	void Initialize(ros::NodeHandle & nh) {
 		this->nh = nh;
 		fov_pub = nh.advertise<visualization_msgs::Marker>("fov", 0);	
 	}

 	void DrawFrustums(std::vector<NanoMapPose> poses);
 	void PublishFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4, bool color_in_fov);

 private:

 	ros::NodeHandle nh;
 	ros::Publisher fov_pub;

};

#endif