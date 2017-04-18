#ifndef FOV_VISUALIZER_H
#define FOV_VISUALIZER_H

#include <Eigen/Dense>
typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
typedef Eigen::Matrix<Scalar, 4, 1> Vector4;
typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

#include <visualization_msgs/Marker.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"

Vector3 applyTransform(Vector3 p, Eigen::Matrix4d transform) {
  Vector4 p_aug;
  p_aug << p, 1.0;
  p_aug = transform * p_aug;
  return Vector3(p_aug(0), p_aug(1), p_aug(2));
}

std_msgs::ColorRGBA GetColorForFOV(int fov_id) {
  fov_id = fov_id + 100;
  std_msgs::ColorRGBA c;
  if (fov_id == 0 ) {
    //purple
    c.r = 0.9;
    c.g = 0.1;
    c.b = 0.9;
    c.a = 0.15;
  }
  else if (fov_id == 10 ) {
    //orange
    c.r = 1.0;
    c.g = 165.0/255.0;
    c.b = 0.1;
    c.a = 0.15;
  }
  else if (fov_id == 20 ) {
    //blue
    c.r = 0.1;
    c.g = 0.1;
    c.b = 1.0;
    c.a = 0.15;
  }
  else if (fov_id == 30) {
    //red
    c.r = 1.0;
    c.g = 0.1;
    c.b = 0.1;
    c.a = 0.15;
  }
  else if (fov_id == 40) {
    //red
    c.r = 0.1;
    c.g = 1.0;
    c.b = 0.1;
    c.a = 0.15;
  }
  else {
    c.r = 0.0;
    c.g = 0.0;
    c.b = 0.0;
    c.a = 0.0;
  }
  return c;
}


void BuildSideOfFOV(Vector3 body, Vector3 corner_1, Vector3 corner_2, visualization_msgs::Marker& marker, int fov_id, bool color_in_fov) {

		geometry_msgs::Point p;
		p.x = body(0);
		p.y = body(1);
  	p.z = body(2);

   		geometry_msgs::Point p2 = p;
   		p2.x = corner_1(0);
   		p2.y = corner_1(1);
   		p2.z = corner_1(2);

   		geometry_msgs::Point p3 = p;
   		p3.x = corner_2(0);
   		p3.y = corner_2(1);
   		p3.z = corner_2(2);

   		marker.points.push_back(p);
   		marker.points.push_back(p2);
   		marker.points.push_back(p3);

      std_msgs::ColorRGBA c = GetColorForFOV(fov_id); 

   		marker.colors.push_back(c);
   		marker.colors.push_back(c);
  		marker.colors.push_back(c);       	    	
	}

void BuildLineOfFOV(Vector3 corner_1, Vector3 corner_2, visualization_msgs::Marker& marker, int fov_id, bool color_in_fov) {

	geometry_msgs::Point p;
	p.x = corner_1(0);
	p.y = corner_1(1);
	p.z = corner_1(2);

	geometry_msgs::Point p2 = p;
	p2.x = corner_2(0);
	p2.y = corner_2(1);
	p2.z = corner_2(2);

	marker.points.push_back(p);
	marker.points.push_back(p2);

	std_msgs::ColorRGBA c = GetColorForFOV(fov_id);
  c.a = 0.50;  
	marker.colors.push_back(c);
	marker.colors.push_back(c);      	    	
}

std::vector<visualization_msgs::Marker> BuildFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4, bool color_in_fov) {
    std::string drawing_frame = "world";
    visualization_msgs::Marker marker;
    marker.header.frame_id = drawing_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "fov_side";
    marker.id = fov_id;
    marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    std::vector<Vector3> fov_corners;
    fov_corners.push_back(corner_1); // bottom right
    fov_corners.push_back(corner_2); // top right
    fov_corners.push_back(corner_3); // top left
    fov_corners.push_back(corner_4); // bottom left    

    int j = 0;
    for (int i = 0; i < 4; i++) {
      j = i+1;
      if (j == 4) {j = 0;}; // connect back around
      BuildSideOfFOV(body, fov_corners.at(i), fov_corners.at(j), marker, fov_id, color_in_fov);
    }
    std::vector<visualization_msgs::Marker> markers;
    markers.push_back(marker);

    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.id = fov_id+1;
    marker.ns = "fov_side";
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.points.clear();
    marker.colors.clear();
    for (int i = 0; i < 4; i++) {
      j = i+1;
      if (j == 4) {j = 0;}; // connect back around
      BuildLineOfFOV(body, fov_corners.at(i), marker, fov_id, color_in_fov); // don't need to rotate 0,0,0
      BuildLineOfFOV(fov_corners.at(i), fov_corners.at(j), marker, fov_id, color_in_fov);
    }
    markers.push_back(marker);
    return markers;
}

#endif