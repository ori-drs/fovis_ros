#ifndef KMCL_FOVISION_HPP_
#define KMCL_FOVISION_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>

#include <fovis/fovis.hpp>

#include <lcmtypes/bot_core/pose_t.hpp>
#include <lcmtypes/fovis/stats_t.hpp>
#include <lcmtypes/fovis/update_t.hpp>

#include <bot_lcmgl_client/lcmgl.h>
#include "visualization.hpp"


class FoVision
{
public:
    FoVision(boost::shared_ptr<lcm::LCM> &lcm_,
             boost::shared_ptr<fovis::StereoCalibration> kcal,
             bool draw_lcmgl_, int which_vo_options_);
    
    
    ~FoVision();

    bool draw_lcmgl_;
    int which_vo_options_;
    
    
    void doOdometry(uint8_t *left_buf,uint8_t *right_buf, int64_t utime);
    void doOdometry(uint8_t *left_buf,float *disparity_buf, int64_t utime);

    void send_status_msg(std::string text);

    fovis::update_t get_delta_translation_msg(Eigen::Isometry3d motion_estimate,
      Eigen::MatrixXd motion_cov, int64_t timestamp, int64_t prev_timestamp);
    
    void send_delta_translation_msg(Eigen::Isometry3d motion_estimate,
      Eigen::MatrixXd motion_cov, std::string channel_name);
    
    void fovis_stats();
    
    Eigen::Isometry3d getMotionEstimate(){ 
      Eigen::Isometry3d motion_estimate = odom_.getMotionEstimate();
      
      // rotate coordinate frame so that look vector is +X, and up is +Z
      Eigen::Matrix3d M;
      M <<  0,  0, 1,
	    -1,  0, 0,
	    0, -1, 0;

      motion_estimate= M * motion_estimate;
      Eigen::Vector3d translation(motion_estimate.translation());
      Eigen::Quaterniond rotation(motion_estimate.rotation());
      rotation = rotation * M.transpose();  
      
      Eigen::Isometry3d motion_estimate_out;
      motion_estimate_out.setIdentity();
      motion_estimate_out.translation() << translation[0],translation[1],translation[2];
      motion_estimate_out.rotate(rotation);
      return motion_estimate_out;
    }
      
    fovis::MotionEstimateStatusCode getEstimateStatus(){
      fovis::MotionEstimateStatusCode estim_status = odom_.getMotionEstimateStatus();
      /*std::cout << estim_status << "is status\n";
      if (estim_status == fovis::SUCCESS){
        std::cout << estim_status << "is valid status\n";
      }else if (estim_status = fovis::INSUFFICIENT_INLIERS){
        std::cout << estim_status << "is FOVIS_UPDATE_T_ESTIMATE_INSUFFICIENT_FEATURES\n";
      }*/

      return estim_status;
    }
    
    const fovis::FeatureMatch* getMatches(){ return odom_.getMotionEstimator()->getMatches(); }
    int getNumMatches(){ return odom_.getMotionEstimator()->getNumMatches(); }
    int getNumInliers(){ return odom_.getMotionEstimator()->getNumInliers(); }

    bool getChangeReferenceFrames(){ return odom_.getChangeReferenceFrames(); }

    void getMotion(Eigen::Isometry3d &delta, Eigen::MatrixXd &delta_cov, fovis::MotionEstimateStatusCode& delta_status ){
      delta=       odom_.getMotionEstimate();
      delta_cov =  odom_.getMotionEstimateCov();
      delta_status = odom_.getMotionEstimateStatus();
    }

    Eigen::Isometry3d getPose(){
      return odom_.getPose();
    }

    void setPublishFovisStats(bool publish_fovis_stats_in){ publish_fovis_stats_ = publish_fovis_stats_in; }

private:
    boost::shared_ptr<lcm::LCM> lcm_;
    boost::shared_ptr<fovis::StereoCalibration> kcal_;
    fovis::VisualOdometry odom_;
    
    // Depth Sources:
    fovis::StereoDepth* stereo_depth_; // typical left/right stereo
    fovis::StereoDisparity* stereo_disparity_; // left/disparity from multisense

    bool publish_fovis_stats_;
    bool publish_pose_;

    //fovis::PrimeSenseDepth depth_producer_; // disparity from Freenect
    //fovis::DepthImage* depth_image_; // depth from OpenNI
    
    float* depth_data_;
    Eigen::Isometry3d pose_;

    fovis::VisualOdometryOptions getOptions();

    void getOptionsHordur(fovis::VisualOdometryOptions &vo_opts);
    void getOptionsCommon(fovis::VisualOdometryOptions &vo_opts);
    void getOptionsFaster(fovis::VisualOdometryOptions &vo_opts);


    int64_t current_timestamp_,prev_timestamp_;

    Visualization* visualization_;    
};

#endif
