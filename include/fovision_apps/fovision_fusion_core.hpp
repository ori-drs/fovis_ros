#include <Eigen/Dense>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>

#include "voconfig/voconfig.hpp"
#include "vofeatures/vofeatures.hpp"
#include "voestimator/voestimator.hpp"
#include "fovision/fovision.hpp"

struct FusionCoreConfig
{
  std::string camera_config; // which block from the cfg to read
  // 0 none, 1 at init, 2 rpy, 2 rp only
  int fusion_mode;
  bool publish_feature_analysis;
  int feature_analysis_publish_period; // number of frames between publishing the point features 
  std::string output_extension;
  std::string output_signal;
  bool output_signal_at_10Hz;
  std::string input_channel;
  bool verbose;
  int correction_frequency;
  std::string imu_channel;
  std::string in_log_fname;
  std::string param_file;
  bool draw_lcmgl;
  bool write_feature_output;
  int which_vo_options;
  bool extrapolate_when_vo_fails;
};


class FusionCore{
  public:
    FusionCore(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const FusionCoreConfig& fcfg_);
    
    ~FusionCore(){
      free (left_buf_);
      free(right_buf_);
      free(depth_buf_);
    }

    bool isPoseInitialized(){
        return pose_initialized_;
    }

    void setCurrentTime(int64_t utime_in){
        utime_prev_ = utime_cur_;
        utime_cur_ = utime_in;
    }

    uint8_t* left_buf_;
    uint8_t* right_buf_;
    mutable std::vector<float> disparity_buf_; // Is mutable necessary?
    uint8_t* rgb_buf_ ;
    uint8_t* decompress_disparity_buf_;    

    float* depth_buf_;    


    void doOdometryLeftRight(){
        vo_->doOdometry(left_buf_,right_buf_, utime_cur_);
    }

    void doOdometryLeftDisparity(){
        vo_->doOdometry(left_buf_,disparity_buf_.data(), utime_cur_);
    }

    void doOdometryLeftDepth(){
        vo_->doOdometryDepthImage(left_buf_,depth_buf_, utime_cur_);
    }

    bool isFilterDisparityEnabled(){
        return filter_disparity_;
    }
    // Filter the disparity image
    void filterDisparity(int w, int h);
    // Republish image to LCM. Used to examine the disparity filtering
    //void republishImage(const  bot_core::images_t* msg);



    void doPostProcessing(){
        updateMotion();

        if (fcfg_.publish_feature_analysis)
            featureAnalysis();
        if (fcfg_.output_signal_at_10Hz)
            outputSignalAt10Hz();

        fuseInterial(local_to_body_orientation_from_imu_, utime_cur_);
    }

    void updateMotion();
    void featureAnalysis();
    void outputSignalAt10Hz();
    void fuseInterial(Eigen::Quaterniond local_to_body_orientation_from_imu, int64_t utime);

    void updatePosition(Eigen::Isometry3d delta_camera){
        estimator_->updatePosition(utime_cur_, utime_prev_, delta_camera);
    }


    Eigen::Quaterniond imuOrientationToRobotOrientation(Eigen::Quaterniond imu_orientation_from_imu);

    void setBodyOrientationFromImu(Eigen::Quaterniond local_to_body_orientation_from_imu, Eigen::Vector3d gyro, int64_t imu_utime);
    Eigen::Quaterniond getBodyOrientationFromImu(){ return local_to_body_orientation_from_imu_; }

    Eigen::Isometry3d getBodyPose(){
        return estimator_->getBodyPose();
    }


  private:
    const FusionCoreConfig fcfg_;    
    
    int image_size_; // just the resolution of the image
    image_io_utils*  imgutils_;    
    
    int64_t utime_cur_, utime_prev_;

    boost::shared_ptr<lcm::LCM> lcm_recv_;
    boost::shared_ptr<lcm::LCM> lcm_pub_;
    BotParam* botparam_;
    BotFrames* botframes_;
    //bot::frames* botframes_cpp_;
    voconfig::KmclConfiguration* config_;

    // Vision and Estimation
    FoVision* vo_;
    VoFeatures* features_;
    uint8_t* left_buf_ref_; // copies of the reference images - probably can be extracted from fovis directly
    int64_t ref_utime_;
    Eigen::Isometry3d ref_camera_pose_; // [pose of the camera when the reference frames changed
    bool changed_ref_frames_;
    VoEstimator* estimator_;

    // IMU
    bool pose_initialized_; // initalized from VO
    int imu_counter_;
    Eigen::Isometry3d body_to_imu_, imu_to_camera_;
    
    // previous successful vo estimates as rates:
    Eigen::Vector3d vo_velocity_linear_;
    Eigen::Vector3d vo_velocity_angular_;
    Eigen::Vector3d camera_linear_velocity_from_imu_; // in camera frame
    Eigen::Vector3d camera_angular_velocity_from_imu_; // in camera frame
    Eigen::Vector3d camera_angular_velocity_from_imu_alpha_; // in camera frame

    // Most recent IMU-derived body orientation
    Eigen::Quaterniond local_to_body_orientation_from_imu_;


    // Image prefiltering
    bool filter_disparity_;
    double filter_disparity_below_threshold_;
    double filter_disparity_above_threshold_;
    int filter_image_rows_above_;
    bool publish_filtered_image_;

};    