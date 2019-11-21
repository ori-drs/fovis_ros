#include "voestimator/voestimator.hpp"

VoEstimator::VoEstimator():
  pose_initialized_(false),
  vo_initialized_(false)
{
  local_to_body_.setIdentity();
  camera_to_body_.setIdentity();
}


void VoEstimator::updatePosition(int64_t utime,
                                 int64_t utime_prev,
                                 const Eigen::Isometry3d& delta_camera)
{
  // 0. Assume head to camera is rigid:
  //camera_to_body_.setIdentity();

  // rosrun tf tf_echo base realsense_d435_front_forward_depth_optical_frame .... in sim - the only optical frame in sim
  // however: realsense_d435_front_forward_infra1_optical_frame is the actual frame in the data
  // TODO: update when robot is running live
  //camera_to_body_.translation().x() = 0.386;
  //camera_to_body_.translation().y() = 0.015;
  //camera_to_body_.translation().z() = 0.160;
  //Eigen::Quaterniond quat = euler_to_quat(-1.780, 0.0, -1.571); //12 degrees pitch down in optical coordinates
  //camera_to_body_.rotate( quat );
  //camera_to_body_ = camera_to_body_.inverse(); //NB
  //REPLACE get_trans_with_utime( botframes_ ,  "body", std::string(camera_config_ + "_LEFT").c_str(), utime, camera_to_body_);


  // 1. Update the Position of the head frame:
  Eigen::Isometry3d delta_body = camera_to_body_.inverse() * delta_camera * camera_to_body_;
  local_to_body_ = local_to_body_ * delta_body;

  // 2. Evaluate Rates:
  double delta_time = static_cast<double>(utime - utime_prev) / 1e6;

  if(utime_prev == 0) {
    std::cout << "utime_prev is zero [at init]\n";
    vo_initialized_ = false; // reconfirming what is set above
  } else {
    vo_initialized_ = true;
    head_lin_rate_ = delta_body.translation() / delta_time;
    Eigen::Vector3d delta_body_rpy;
    quat_to_euler(Eigen::Quaterniond(delta_body.rotation()), delta_body_rpy(0), delta_body_rpy(1), delta_body_rpy(2));
    head_rot_rate_ = delta_body_rpy / delta_time; // rotation rate
  }    
  
  // 3. Maintain a smoothed version:
  double alpha = 0.8;
  head_lin_rate_alpha_ =  alpha * head_lin_rate_alpha_ + (1 - alpha) * head_lin_rate_;
  head_rot_rate_alpha_ =  alpha * head_rot_rate_alpha_ + (1 - alpha) * head_rot_rate_;

  local_to_body_prev_ = local_to_body_;
  delta_body_prev_ = delta_body;
}
  
