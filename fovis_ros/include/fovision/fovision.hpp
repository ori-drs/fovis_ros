#ifndef KMCL_FOVISION_HPP_
#define KMCL_FOVISION_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <fovis/fovis.hpp>

#include "visualization.hpp"

class FoVision
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  using StereoCalibrationPtr = std::shared_ptr<fovis::StereoCalibration>;

public:
    FoVision(const StereoCalibrationPtr& kcal,
             int which_vo_options_);
    
    
    virtual ~FoVision();


    
    void writeRawImage(float *float_buf, int width, int height, int64_t utime);
    
    void doOdometry(uint8_t *left_buf, uint8_t *right_buf, int64_t utime);

    void doOdometry(uint8_t *left_buf, float *disparity_buf, int64_t utime);

    void doOdometryDepthImage(uint8_t *left_buf, float *depth_buf, int64_t utime);

    void send_delta_translation_msg(Eigen::Isometry3d motion_estimate,
                                    Eigen::MatrixXd motion_cov,
                                    std::string channel_name);
    
    void fovis_stats();
    
    inline Eigen::Isometry3d getMotionEstimate(){
      return odom_->getMotionEstimate();
    }
      
    inline fovis::MotionEstimateStatusCode getEstimateStatus(){
      return odom_->getMotionEstimateStatus();
    }
    
    inline const fovis::FeatureMatch* getMatches(){
      return odom_->getMotionEstimator()->getMatches();
    }

    inline int getNumMatches(){
      return odom_->getMotionEstimator()->getNumMatches();
    }

    inline int getNumInliers(){
      return odom_->getMotionEstimator()->getNumInliers();
    }

    inline bool getChangeReferenceFrames(){
      return odom_->getChangeReferenceFrames();
    }

    inline void getMotion(Eigen::Isometry3d &delta,
                          Eigen::MatrixXd &delta_cov,
                          fovis::MotionEstimateStatusCode& delta_status)
    {
      delta = odom_->getMotionEstimate();
      delta_cov =  odom_->getMotionEstimateCov();
      delta_status = odom_->getMotionEstimateStatus();
    }

    inline Eigen::Isometry3d getPose() {
      return odom_->getPose();
    }

    inline void setPublishFovisStats(bool publish_fovis_stats_in){
      publish_fovis_stats_ = publish_fovis_stats_in;
    }

    inline const fovis::VisualOdometry* getVisualOdometry() const {
      return odom_;
    }

private:
    int which_vo_options_;
    StereoCalibrationPtr kcal_;
    fovis::VisualOdometry* odom_;
    
    // Depth Sources:
    fovis::StereoDepth* stereo_depth_; // typical left/right stereo
    fovis::StereoDisparity* stereo_disparity_; // left/disparity from multisense
    fovis::DepthImage* depth_image_; // left/depth from realsense

    bool publish_fovis_stats_;
    bool publish_pose_;

    Eigen::Isometry3d pose_;

    fovis::VisualOdometryOptions getOptions();

    void getOptionsHordur(fovis::VisualOdometryOptions &vo_opts);
    void getOptionsCommon(fovis::VisualOdometryOptions &vo_opts);
    void getOptionsFaster(fovis::VisualOdometryOptions &vo_opts);

    uint64_t current_timestamp_;
    uint64_t prev_timestamp_;

    Visualization* visualization_;    
};

#endif
