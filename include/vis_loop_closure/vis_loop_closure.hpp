#ifndef vis_loop_closure_HPP_
#define vis_loop_closure_HPP_

#include <lcm/lcm-cpp.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <estimate_pose/pose_estimator.hpp>

#include <pronto_vis/pronto_vis.hpp>

#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include "voconfig/voconfig.hpp"
#include "vofeatures/vofeatures.hpp"


struct VisLoopClosureConfig
{
  int min_inliers; // 60 used by Hordur, might want to use a higher number
  bool verbose;
  bool publish_diagnostics;
  bool use_cv_show; // use open cv show popup (blocks thread)
  std::string param_file;
};


struct FrameMatch{
  std::vector<int> featuresA_indices;
  std::vector<int> featuresB_indices;

  std::vector<ImageFeature> featuresA;
  std::vector<ImageFeature> featuresB;
  
  pose_estimator::PoseEstimateStatus status; 
  // Modes: SUCCESS, INSUFFICIENT_INLIERS,  OPTIMIZATION_FAILURE, REPROJECTION_ERROR
  Eigen::Isometry3d delta; // A->B transform : where is B relative to A
  int n_inliers;
  int n_registration_inliers; // number of inliers retried from the transfromation estimation
  // TODO: not sure if these are different 

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
typedef boost::shared_ptr<FrameMatch> FrameMatchPtr;


class VisLoopClosure
{
  public:
    typedef boost::shared_ptr<VisLoopClosure> Ptr;
    typedef boost::shared_ptr<const VisLoopClosure> ConstPtr;
    
    VisLoopClosure (boost::shared_ptr<lcm::LCM> &lcm_, const VisLoopClosureConfig& reg_cfg_);

    void read_features(std::string fname,
        std::vector<ImageFeature>& features);
    
    void align_images(cv::Mat &img0, cv::Mat &img1, 
                           std::vector<ImageFeature> &features0, std::vector<ImageFeature> &features1,
                           int64_t utime0, int64_t utime1, FrameMatchPtr &match);
    
    void send_both_reg(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
        Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1,
        int64_t utime0, int64_t utime1           );
    
    void send_both_reg_inliers(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
        Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1,
        std::vector<int> feature_inliers0,    std::vector<int> feature_inliers1 ,
        int64_t utime0, int64_t utime1);
    
    void send_lcm_image(cv::Mat &img, std::string channel );

    void getFilenames(std::string path_to_folder, std::vector<std::string> &futimes, 
        std::vector<std::string> &utimes_strings);

  private:
    const VisLoopClosureConfig reg_cfg_;    

    boost::shared_ptr<lcm::LCM> lcm_;

    pronto_vis* pc_vis_;

    VoFeatures* features_;   
    BotParam* botparam_;
    BotFrames* botframes_;
    bot::frames* botframes_cpp_;
    voconfig::KmclConfiguration* config_;


    image_io_utils*  imgutils_;

    Eigen::Matrix<double, 3, 4> projection_matrix_;

};

#endif
