#include "voestimator/voestimator.hpp"

VoEstimator::VoEstimator():
  pose_initialized_(false),
  vo_initialized_(false)
{
  world_to_body_curr_.setIdentity();
  camera_to_body_.setIdentity();
}


void VoEstimator::updatePose(int64_t utime_curr,
                             int64_t utime_prev,
                             const Eigen::Isometry3d& delta_camera)
{
  // 0. update times
  utime_curr_ = utime_curr;
  utime_prev_ = utime_prev;

  // 1. Update the Position of the head frame:
  delta_body_curr_ = camera_to_body_.inverse() * delta_camera * camera_to_body_;
  world_to_body_curr_ = world_to_body_prev_ * delta_body_curr_;

  // 2. Evaluate Rates:
  double delta_time = static_cast<double>(utime_curr_ - utime_prev_) / 1e6;

  if(utime_prev == 0) {
    std::cout << "utime_prev is zero [at init] " << std::endl;
    vo_initialized_ = false; // reconfirming what is set above
  } else {
    vo_initialized_ = true;
    head_lin_rate_ = delta_body_curr_.translation() / delta_time;
    Eigen::Vector3d delta_body_rpy;
    quat_to_euler(Eigen::Quaterniond(delta_body_curr_.rotation()), delta_body_rpy(0), delta_body_rpy(1), delta_body_rpy(2));
    head_rot_rate_ = delta_body_rpy / delta_time; // rotation rate
  }    
  
  // 3. Maintain a smoothed version:
  head_lin_rate_alpha_ =  alpha * head_lin_rate_alpha_ + (1 - alpha) * head_lin_rate_;
  head_rot_rate_alpha_ =  alpha * head_rot_rate_alpha_ + (1 - alpha) * head_rot_rate_;

  world_to_body_prev_ = world_to_body_curr_;
  delta_body_prev_ = delta_body_curr_;
}
  
