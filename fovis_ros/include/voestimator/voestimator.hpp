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
 * @brief A VO-based non-probablistic state estimator for a stereo camera
 * (tested on Carnegie Robotics' MultiSense SL and Intel's RealSense D435)
 * - occasionally uses IMU to avoid orientation drift
 * - when VO fails extrapolate using previous vision lin rate and imu rot rates
 * For IMU orientation integration:
 * Estimate is maintained in the body frame which is assumed to be
 * Forward-Left-Up such at roll and pitch can be isolated from yaw.
 */
class VoEstimator
{
public:
  VoEstimator();
  virtual ~VoEstimator() = default;

  void updatePose(int64_t utime_curr,
                  int64_t utime_prev,
                  const Eigen::Isometry3d& delta_camera);

  inline void getBodyRelativePose(uint64_t& utime_prev,
                                  uint64_t& utime_curr,
                                  Eigen::Isometry3d& relative_pose) {
    utime_prev = utime_prev_;
    utime_curr = utime_curr_;
    relative_pose = delta_body_curr_;
  }

  inline Eigen::Isometry3d getCameraPose(){
    return world_to_body_curr_ * camera_to_body_.inverse();
  }

  inline Eigen::Isometry3d getBodyPose(){
    return world_to_body_curr_;
  }

  void setBodyPose(const Eigen::Isometry3d& world_to_body) {
    world_to_body_curr_ = world_to_body;
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
  bool pose_initialized_ = false;
  bool vo_initialized_ = false;

  Eigen::Isometry3d camera_to_body_  = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d world_to_body_curr_ = Eigen::Isometry3d::Identity();
  
  Eigen::Isometry3d world_to_body_prev_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d delta_body_prev_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d delta_body_curr_ = Eigen::Isometry3d::Identity();

  uint64_t utime_curr_ = 0;
  uint64_t utime_prev_ = 0;
  
  // Cache of rates: All are stored as RPY
  double alpha = 0.8;
  Eigen::Vector3d head_rot_rate_       = Eigen::Vector3d::Zero();
  Eigen::Vector3d head_lin_rate_       = Eigen::Vector3d::Zero();
  Eigen::Vector3d head_rot_rate_alpha_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d head_lin_rate_alpha_ = Eigen::Vector3d::Zero();
};

#endif
