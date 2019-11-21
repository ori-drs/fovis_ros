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

/**
 * @brief A simple class which maintains an estimate of a head position by
 * integrating the delta from a Visual Odometry source
 */
class VoEstimator
{
public:
  VoEstimator();
  virtual ~VoEstimator() = default;

  void updatePosition(int64_t utime,
                      int64_t utime_prev,
                      const Eigen::Isometry3d& delta_camera);

  inline Eigen::Isometry3d getCameraPose(){
    return local_to_body_*camera_to_body_.inverse();
  }

  inline Eigen::Isometry3d getBodyPose(){
    return local_to_body_;
  }

  void setBodyPose(const Eigen::Isometry3d& local_to_body_in) {
    local_to_body_ = local_to_body_in;
    pose_initialized_ = true;
  }

  inline Eigen::Vector3d getHeadLinearRate(){
    return head_lin_rate_;
  }

  inline Eigen::Vector3d getHeadRotationRate(){
    return head_rot_rate_;
  }

  inline void setCameraToBody(const Eigen::Isometry3d& camera_to_body_in) {
    camera_to_body_ = camera_to_body_in;
  }


private:
  // have we received the first pose estimate:?
  bool pose_initialized_;
  bool vo_initialized_;

  Eigen::Isometry3d camera_to_body_;
  Eigen::Isometry3d local_to_body_;
  
  Eigen::Isometry3d local_to_body_prev_;
  Eigen::Isometry3d delta_body_prev_;
  
  // Cache of rates: All are stored as RPY
  Eigen::Vector3d head_rot_rate_, head_lin_rate_;
  Eigen::Vector3d head_rot_rate_alpha_, head_lin_rate_alpha_;
  
};

#endif
