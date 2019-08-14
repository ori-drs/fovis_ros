#ifndef VOESTIMATOR_HPP_
#define VOESTIMATOR_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <fovision/common.hpp>

//#include <lcm/lcm-cpp.hpp>
//#include <bot_frames/bot_frames.h>
////#include <bot_frames_cpp/bot_frames_cpp.hpp>
//#include <lcmtypes/bot_core.hpp>

//#include <pronto_vis/pronto_vis.hpp> // visualize pt clds


// A simple class which maintains an estimate of a head position
// and can publish it to LCM
class VoEstimator
{
public:
  //VoEstimator(boost::shared_ptr<lcm::LCM> &lcm_, BotFrames* botframes_, 
  VoEstimator(std::string channel_extension_ = "", std::string camera_config_ = "CAMERA");
  ~VoEstimator();

  void updatePosition(int64_t utime, int64_t utime_prev, Eigen::Isometry3d delta_camera);

  Eigen::Isometry3d getCameraPose(){ return local_to_body_*camera_to_body_.inverse(); }
  Eigen::Isometry3d getBodyPose(){ return local_to_body_; }
  void setBodyPose(Eigen::Isometry3d local_to_body_in){
    local_to_body_ = local_to_body_in;
    pose_initialized_ = true;
  }

  Eigen::Vector3d getHeadLinearRate(){ return head_lin_rate_; }
  Eigen::Vector3d getHeadRotationRate(){ return head_rot_rate_; }

  void publishPose(int64_t utime, std::string channel, Eigen::Isometry3d pose,
                   Eigen::Vector3d vel_lin, Eigen::Vector3d vel_ang);
  void publishUpdate(int64_t utime, Eigen::Isometry3d local_to_head, std::string channel, bool output_alpha_filter);

private:
  //boost::shared_ptr<lcm::LCM> lcm_;
  //pronto_vis* pc_vis_;
  
  // have we received the first pose estimate:?
  bool pose_initialized_;
  bool vo_initialized_;

  //BotFrames* botframes_;

  std::string channel_extension_;
  std::string camera_config_;
  Eigen::Isometry3d camera_to_body_;
  Eigen::Isometry3d local_to_body_;
  
  Eigen::Isometry3d local_to_body_prev_;
  Eigen::Isometry3d delta_body_prev_;
  
  // Cache of rates: All are stored as RPY
  Eigen::Vector3d head_rot_rate_, head_lin_rate_;
  Eigen::Vector3d head_rot_rate_alpha_, head_lin_rate_alpha_;
  
};

#endif
