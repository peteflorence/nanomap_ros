#include "fov_evaluator.h"

Vector3 FovEvaluator::RotateToSensorFrame(Vector3 position_body_frame) {
  if(0){std::cout << "rotate matrix " << R_body_to_rdf_ << std::endl;}
  return R_body_to_rdf_*position_body_frame;
}

bool FovEvaluator::IsBehind(Vector3 position) const {
  return (position(2) < -0.5);
}

bool FovEvaluator::IsBeyondSensorHorizon(Vector3 position) const {
  return (position(2) > 10.0);
}

bool FovEvaluator::IsOutsideDeadBand(Vector3 position) const {
  return (position.squaredNorm() > 0.5);
}

NanoMapFovStatus FovEvaluator::EvaluateFov(PointCloudPtr const& point_cloud_ptr, Vector3 position) const {
    if (IsBehind(position)) {
      return NanoMapFovStatus::behind;
    }
    if (IsOutsideDeadBand(position)) {
      return NanoMapFovStatus::free_space;
    }

    Vector3 projected = K * position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    // Checks if outside left/right FOV
    if ( (pi_x < 0) || (pi_x > (num_x_pixels - 1)) ) {
      return NanoMapFovStatus::laterally_outside_fov;
    }
    // Checks if above top/bottom FOV
    if (pi_y < 0) {
      return NanoMapFovStatus::laterally_outside_fov; 
    }
    if (pi_y > (num_y_pixels - 1)) {
      return NanoMapFovStatus::laterally_outside_fov; 
    }

    //Checks for occlusion
    if (point_cloud_ptr == nullptr) {
      return NanoMapFovStatus::free_space;
    } 
    pcl::PointXYZ point = point_cloud_ptr->at(pi_x,pi_y);
    if (std::isnan(point.z)) { 
       return NanoMapFovStatus::free_space;
    }
    if( position(2) >  point.z ) {
      return NanoMapFovStatus::occluded;
    }
    return NanoMapFovStatus::free_space;
}

void FovEvaluator::SetCameraInfo(double bin, double width, double height, Matrix3 const& K_camera_info) {
  if (bin < 1.0) {binning = 1.0;}
  else {binning = bin;}
  num_x_pixels = width / binning;
  num_y_pixels = height / binning;
  K = K_camera_info;
  K /= binning;
  K(2,2) = 1.0;
  return;
}

void FovEvaluator::SetBodyToRdf(Matrix3 const& R_body_to_rdf) {
    R_body_to_rdf_ = R_body_to_rdf;
}