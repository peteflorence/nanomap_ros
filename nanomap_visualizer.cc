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
  Vector3 corner_0 = applyTransform(body, body_to_world); // don't need to rotate 0,0,0
  Vector3 corner_1 = applyTransform(bottom_right, body_to_world);
  Vector3 corner_2 = applyTransform(top_right, body_to_world);
  Vector3 corner_3 = applyTransform(top_left, body_to_world);
  Vector3 corner_4 = applyTransform(bottom_left, body_to_world);

  PublishFovMarker(0, corner_0, corner_1, corner_2, corner_3, corner_4, true);

  Matrix4 transform_so_far = body_to_world;

	for (int i = 0; i < num_edges; i++) {
	  transform_so_far = transform_so_far * invertTransform(edges.at(i));

	  corner_0 = applyTransform(body, transform_so_far); // don't need to rotate 0,0,0
    corner_1 = applyTransform(bottom_right, transform_so_far);
    corner_2 = applyTransform(top_right, transform_so_far);
    corner_3 = applyTransform(top_left, transform_so_far);
    corner_4 = applyTransform(bottom_left, transform_so_far);
   	if (i % 5 == 0){PublishFovMarker((i+1)*2, corner_0, corner_1, corner_2, corner_3, corner_4, false);}
	}

}


void NanoMapVisualizer::PublishFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4, bool color_in_fov) {
    std::vector<visualization_msgs::Marker> markers = BuildFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
    fov_pub.publish( markers.at(0) );
    fov_pub.publish( markers.at(1) );
}
