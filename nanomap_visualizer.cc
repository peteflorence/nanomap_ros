#include "nanomap_visualizer.h"
#include "frustum_visualization.h"

void NanoMapVisualizer::DrawFrustums(std::vector<Matrix4> edges) {

	// iterate over poses
	  int fov_id = 0;

      // FIND CORNERS

	  Matrix3 BodyToRDF_inverse;
	  BodyToRDF_inverse << 0, 0, 1, -1, 0, 0, 0, -1, 0;

      // start in that poses' rdf, and rotate to body
      Vector3 bottom_right = BodyToRDF_inverse * Vector3(7,5.25,10);
      Vector3 top_right = BodyToRDF_inverse * Vector3(7,-5.25,10);
      Vector3 top_left = BodyToRDF_inverse * Vector3(-7,-5.25,10);
      Vector3 bottom_left = BodyToRDF_inverse * Vector3(-7,5.25,10);
      Vector3 body = Vector3(0,0,0);

      //Eigen::Matrix4d transform = transformFromPreviousBodyToWorld(fov_id);
      Matrix4 transform;
      transform.setIdentity();
      body = applyTransform(body, transform); // don't need to rotate 0,0,0
      Vector3 corner_1 = applyTransform(bottom_right, transform);
      Vector3 corner_2 = applyTransform(top_right, transform);
      Vector3 corner_3 = applyTransform(top_left, transform);
      Vector3 corner_4 = applyTransform(bottom_left, transform);

      PublishFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, false);
}


void NanoMapVisualizer::PublishFovMarker(int fov_id, Vector3 body, Vector3 corner_1, Vector3 corner_2, Vector3 corner_3, Vector3 corner_4, bool color_in_fov) {
  std::vector<visualization_msgs::Marker> markers = BuildFovMarker(fov_id, body, corner_1, corner_2, corner_3, corner_4, color_in_fov);
  fov_pub.publish( markers.at(0) );
  fov_pub.publish( markers.at(1) );
}
