#include "nanomap_visualizer.h"
#include "frustum_visualization.h"

void NanoMapVisualizer::SetLastPose(Matrix4 pose) {
	last_pose = pose;
}

Eigen::Matrix4d invertTransform(Eigen::Matrix4d transform) {
  Matrix3 R = transform.block<3,3>(0,0);
  Vector3 t = transform.block<3,1>(0,3);
  Eigen::Matrix4d inverted_transform = Eigen::Matrix4d::Identity();
  inverted_transform.block<3,3>(0,0) = R.transpose();
  inverted_transform.block<3,1>(0,3) = -1.0 * R.transpose() * t;
  return inverted_transform;
}

void NanoMapVisualizer::DrawFrustums(std::vector<Matrix4> edges) {

	int num_edges = edges.size();

	Matrix3 BodyToRDF_inverse;
  BodyToRDF_inverse << 0, 0, 1, -1, 0, 0, 0, -1, 0;

  // start in that poses' rdf, and rotate to body
  Vector3 bottom_right = BodyToRDF_inverse * Vector3(7,5.25,10);
  Vector3 top_right = BodyToRDF_inverse * Vector3(7,-5.25,10);
  Vector3 top_left = BodyToRDF_inverse * Vector3(-7,-5.25,10);
  Vector3 bottom_left = BodyToRDF_inverse * Vector3(-7,5.25,10);
  Vector3 body = Vector3(0,0,0);

  //Eigen::Matrix4d transform = transformFromPreviousBodyToWorld(fov_id);
  Matrix4 body_to_world = last_pose;
  body = applyTransform(body, body_to_world); // don't need to rotate 0,0,0
  Vector3 corner_1 = applyTransform(bottom_right, body_to_world);
  Vector3 corner_2 = applyTransform(top_right, body_to_world);
  Vector3 corner_3 = applyTransform(top_left, body_to_world);
  Vector3 corner_4 = applyTransform(bottom_left, body_to_world);

  PublishFovMarker(0, body, corner_1, corner_2, corner_3, corner_4, true);

	for (int i = 0; i < 20; i++) {
		//transform = invertTransform(edges.at(i));
		// body = applyTransform(body, transform); // don't need to rotate 0,0,0
  // 	corner_1 = applyTransform(corner_1, transform);
  // 	corner_2 = applyTransform(corner_2, transform);
  // 	corner_3 = applyTransform(corner_3, transform);
  // 	corner_4 = applyTransform(corner_4, transform);
  	PublishFovMarker(i+1, body, corner_1, corner_2, corner_3, corner_4, false);
	}

}


void NanoMapVisualizer::PublishFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4, bool color_in_fov) {
  std::vector<visualization_msgs::Marker> markers = BuildFovMarker(fov_id*2, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
  fov_pub.publish( markers.at(0) );
  fov_pub.publish( markers.at(1) );
}
