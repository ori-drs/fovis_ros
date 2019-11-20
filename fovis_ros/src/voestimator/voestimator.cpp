#include "voestimator/voestimator.hpp"

VoEstimator::VoEstimator(std::string channel_extension_, std::string camera_config_):
  channel_extension_(channel_extension_), camera_config_(camera_config_),
  pose_initialized_(false), vo_initialized_(false){
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

  // 4. Output the head position update, at the rates provided by VO
  publishUpdate(utime_prev, local_to_body_, "POSE_HEAD_ALT_FOVIS", true);
  local_to_body_prev_ = local_to_body_;
  delta_body_prev_ = delta_body;
}
  
void VoEstimator::publishUpdate(int64_t utime,
                                Eigen::Isometry3d local_to_head, std::string channel, bool output_alpha_filter){
  if ((!pose_initialized_) || (!vo_initialized_)) {
    std::cout << (int) pose_initialized_ << " pose | " << (int) vo_initialized_ << " vo - ";
    std::cout << "pose or vo not initialized, refusing to publish POSE_HEAD and POSE_BODY\n";
    return;
  }

  // Send vo pose to collections:
  //Isometry3dTime local_to_headT = Isometry3dTime(utime, local_to_head);
  //pc_vis_->pose_to_lcm_from_list(60000, local_to_headT);
  // std::cout << head_rot_rate_.transpose() << " head rot rate out\n";
  // std::cout << head_lin_rate_.transpose() << " head lin rate out\n";

  // publish local to head pose
  if (output_alpha_filter){
    publishPose(utime, channel + channel_extension_, local_to_head, head_lin_rate_alpha_, head_rot_rate_alpha_);
  }else{
    publishPose(utime, channel + channel_extension_, local_to_head, head_lin_rate_, head_rot_rate_);
  }
}


void VoEstimator::publishPose(int64_t utime, std::string channel, Eigen::Isometry3d pose,
  Eigen::Vector3d vel_lin, Eigen::Vector3d vel_ang){
  Eigen::Quaterniond r(pose.rotation());
  /*
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();
  pose_msg.orientation[0] =  r.w();
  pose_msg.orientation[1] =  r.x();
  pose_msg.orientation[2] =  r.y();
  pose_msg.orientation[3] =  r.z();
  pose_msg.vel[0] = vel_lin(0);
  pose_msg.vel[1] = vel_lin(1);
  pose_msg.vel[2] = vel_lin(2);
  pose_msg.rotation_rate[0] = vel_ang(0);
  pose_msg.rotation_rate[1] = vel_ang(1);
  pose_msg.rotation_rate[2] = vel_ang(2);
  pose_msg.accel[0]=0; // not estimated or filled in
  pose_msg.accel[1]=0;
  pose_msg.accel[2]=0;
  lcm_->publish( channel, &pose_msg);
  */
}
