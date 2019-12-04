#include <Eigen/Dense>

#include "voconfig/voconfig.hpp"
#include "vofeatures/vofeatures.hpp"
#include "voestimator/voestimator.hpp"
#include "fovision/fovision.hpp"

struct FusionCoreConfig
{
  // yaml file to be read
  std::string config_filename = "";

  // how should we fuse IMU sensors? 0 no fusion, 1 at init, 2 rpy, 2 rp only
  int orientation_fusion_mode = 0;

  // how should we set the initial pose? 0 using the config file, 1 using imu, 2 using a pose source
  int initial_pose_mode = 0;
  Eigen::Isometry3d initial_pose = Eigen::Isometry3d::Identity();

  bool publish_feature_analysis = false;

  // number of frames between publishing the point features
  int feature_analysis_publish_period = 1; // 5

  bool verbose = false;
  // was typicall unused at 100;
  int correction_frequency = 1;

  bool write_feature_output = false;
  int which_vo_options = 2;
  bool extrapolate_when_vo_fails = false;
};


class FusionCore{
  public:
    FusionCore(const FusionCoreConfig& fcfg);
    
    virtual ~FusionCore();

    inline bool isPoseInitialized() {
        return pose_initialized_;
    }

    inline void setCurrentTime(int64_t utime_in) {
        utime_prev_ = utime_cur_;
        utime_cur_ = utime_in;
    }

    uint8_t* left_buf_;
    uint8_t* right_buf_;
    mutable std::vector<float> disparity_buf_; // Is mutable necessary?
    uint8_t* rgb_buf_ ;
    uint8_t* decompress_disparity_buf_;    

    float* depth_buf_;    


    inline void doOdometryLeftRight() {
        vo_->doOdometry(left_buf_,right_buf_, utime_cur_);
    }

    inline void doOdometryLeftDisparity() {
        vo_->doOdometry(left_buf_,disparity_buf_.data(), utime_cur_);
    }

    inline void doOdometryLeftDepth() {
        vo_->doOdometryDepthImage(left_buf_,depth_buf_, utime_cur_);
    }

    inline bool isFilterDisparityEnabled() {
        return filter_disparity_;
    }
    // Filter the disparity image
    void filterDisparity(int w, int h);


    inline bool isFilterDepthEnabled() {
        return filter_depth_;
    }
    // Filter the depth image
    void filterDepth(int w, int h);


    inline void doPostProcessing() {
      updateMotion();

      if (fcfg_.publish_feature_analysis){
        featureAnalysis();
      }

      // only use imu after its been initialized
      if (local_to_body_orientation_from_imu_initialized_){
        fuseInertial(local_to_body_orientation_from_imu_, utime_cur_);
      }
    }

    void updateMotion();

    void featureAnalysis();

    inline uint8_t* getFeaturesImage(){
      return features_->getFeaturesImage();
    }

    inline pcl::PointCloud<pcl::PointXYZRGB> getFeaturesCloud() {
      return features_->getFeaturesCloud();
    }

    void writePoseToFile(const Eigen::Isometry3d& pose, int64_t utime);

    void fuseInertial(const Eigen::Quaterniond& local_to_body_orientation_from_imu, int64_t utime);

    inline void updatePosition(const Eigen::Isometry3d& delta_camera) {
        estimator_->updatePose(utime_cur_, utime_prev_, delta_camera);
    }


    Eigen::Quaterniond imuOrientationToRobotOrientation(const Eigen::Quaterniond& imu_orientation_from_imu);

    void setBodyOrientationFromImu(const Eigen::Quaterniond& local_to_body_orientation_from_imu,
                                   const Eigen::Vector3d& gyro, int64_t imu_utime);

    inline Eigen::Quaterniond getBodyOrientationFromImu(){
      return local_to_body_orientation_from_imu_;
    }

    inline Eigen::Isometry3d getBodyPose() {
        return estimator_->getBodyPose();
    }

    inline void initializePose(const Eigen::Isometry3d& init_pose) {
        estimator_->setBodyPose(init_pose);
        pose_initialized_ = true;
        std::cout << "Initialised pose\n";
    }

    inline const fovis::VisualOdometry* getVisualOdometry() const {
      return vo_->getVisualOdometry();
    }

    inline void getBodyRelativePose(uint64_t& utime_prev,
                                    uint64_t& utime_curr,
                                    Eigen::Isometry3d& relative_pose) {
      estimator_->getBodyRelativePose(utime_prev,utime_curr, relative_pose);
    }

  private:
    const FusionCoreConfig fcfg_;    
    
    int image_size_; // just the resolution of the image
    
    int64_t utime_cur_, utime_prev_;

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

    bool local_to_body_orientation_from_imu_initialized_; // has an imu measurement been received
    
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

    bool filter_depth_;
    double filter_depth_below_threshold_;
    double filter_depth_above_threshold_;

    int filter_image_rows_above_;
    bool publish_filtered_image_;

    int counter = 0;
};    
