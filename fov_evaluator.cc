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

NanoMapFovStatus FovEvaluator::EvaluateFov(PointCloudPtr const& point_cloud_ptr, Vector3 position, Vector3 aabb, bool ignore_horizon) const {
    for (int i =0; i < 3; i++) {
      if (aabb(i) < 0) {aabb(i) = -aabb(i);}
    }

    Vector3 behind_aabb = position + Vector3(0,0,-aabb(2));
    Vector3 beyond_aabb = position + Vector3(0,0,aabb(2));
    if (IsBehind(behind_aabb)) {
      return NanoMapFovStatus::behind;
    }
    if (!IsOutsideDeadBand(position) && ignore_horizon) {
      return NanoMapFovStatus::free_space;
    }
    if (!ignore_horizon) {
      if (IsBeyondSensorHorizon(beyond_aabb)) {
        return NanoMapFovStatus::beyond_sensor_horizon;
      }
    }

    Vector3 projected = K * position;
    int pi_x = projected(0)/projected(2); 
    int pi_y = projected(1)/projected(2);

    Vector3 projected_left_down_aabb = K * (position + Vector3(-aabb(0),  aabb(1), 0));
    Vector3 projected_right_up_aabb  = K * (position + Vector3( aabb(0), -aabb(1), 0));
    
    int pi_x_left_down_aabb = projected_left_down_aabb(0)/projected_left_down_aabb(2);
    int pi_y_left_down_aabb = projected_left_down_aabb(1)/projected_left_down_aabb(2);

    int pi_x_right_up_aabb = projected_right_up_aabb(0)/projected_right_up_aabb(2);
    int pi_y_right_up_aabb = projected_right_up_aabb(1)/projected_right_up_aabb(2);

    // Checks if outside left/right FOV
    // std::cout << std::endl;
    // std::cout << "aabb " << aabb.transpose() << std::endl;
    // std::cout << pi_x << " " << pi_x_left_down_aabb << " " << pi_x_right_up_aabb << std::endl;
    // std::cout << pi_y << " " << pi_y_left_down_aabb << " " << pi_y_right_up_aabb << std::endl;
    // std::cout << std::endl;

    if ( (std::min(pi_x_left_down_aabb,pi_x) < 0) || (std::max(pi_x_right_up_aabb,pi_x) > (num_x_pixels - 1)) ) {
      return NanoMapFovStatus::laterally_outside_fov;
    }
    // Checks if above top/bottom FOV
    if (std::min(pi_y_right_up_aabb, pi_y) < 0) {
      return NanoMapFovStatus::laterally_outside_fov; 
    }
    if (std::max(pi_y_left_down_aabb, pi_y) > (num_y_pixels - 1)) {
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