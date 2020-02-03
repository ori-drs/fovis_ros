#include "fovision/fovision.hpp"
#include <iostream>
#include <fstream>


using namespace std;

FoVision::FoVision(const StereoCalibrationPtr& kcal,
                   int which_vo_options_) :
  kcal_(kcal),
  which_vo_options_(which_vo_options_),
  pose_(Eigen::Isometry3d::Identity()),
  publish_fovis_stats_(false),
  publish_pose_(false)
{
  odom_ = new fovis::VisualOdometry(kcal_->getLeftRectification(), FoVision::getOptions());

  fovis::VisualOdometryOptions vo_opts = getOptions();
  // typical left/right stereo
  stereo_depth_ = new fovis::StereoDepth(kcal_.get(), vo_opts);
  // left/disparity from multisense
  stereo_disparity_= new fovis::StereoDisparity( kcal_.get());
  // left/depth from realsense
  depth_image_ = new fovis::DepthImage(kcal_->getRectifiedParameters(), kcal_->getWidth(), kcal_->getHeight());
}


FoVision::~FoVision()
{
  delete stereo_depth_;
  delete stereo_disparity_;
  delete depth_image_;
  delete visualization_;
}


// Typical Stereo:
void FoVision::doOdometry(uint8_t *left_buf,uint8_t *right_buf, int64_t utime){
  prev_timestamp_ = current_timestamp_;
  current_timestamp_ = utime;
  stereo_depth_->setRightImage(right_buf);
  odom_->processFrame(left_buf, stereo_depth_);
}

// Left and Disparity:
void FoVision::doOdometry(uint8_t *left_buf,float *disparity_buf, int64_t utime){
  prev_timestamp_ = current_timestamp_;
  current_timestamp_ = utime;
  stereo_disparity_->setDisparityData(disparity_buf);
  odom_->processFrame(left_buf, stereo_disparity_);
}


void FoVision::writeRawImage(float *float_buf, int width, int height, int64_t utime){
  std::stringstream ss_fname;
  ss_fname << "/tmp/" << utime << "_raw_image_float.raw";

  std::ofstream ofile ( ss_fname.str().c_str() );
  for(int v=0; v<height; v++) { // t2b
    std::stringstream ss;
    for(int u=0; u<width; u++ ) {  //l2r
      ss << float_buf[u + v*width] << ", ";
    }
    ofile << ss.str() << "\n";
  }
  ofile.close();
}


// Left and Depth:
void FoVision::doOdometryDepthImage(uint8_t *left_buf,float *depth_buf, int64_t utime){
  prev_timestamp_ = current_timestamp_;
  current_timestamp_ = utime;
  depth_image_->setDepthImage(depth_buf);
  odom_->processFrame(left_buf, depth_image_);
}


fovis::VisualOdometryOptions FoVision::getOptions() {
  fovis::VisualOdometryOptions vo_opts = fovis::VisualOdometry::getDefaultOptions();

  switch(which_vo_options_){
  case 0:
    // not commonly used. legacy options
    getOptionsHordur(vo_opts);
    break;
  case 1:
    // setting very commonly used
    getOptionsCommon(vo_opts);
    break;
  case 2:
    // modify some of the common parameters to run Multisense at full framerate
    getOptionsCommon(vo_opts);
    getOptionsFaster(vo_opts);
    break;
  default:
    std::cout << "Choose a valid set of VO options e.g. 1 for default options\n";
    exit(-1);
  }
  return vo_opts;
}


void FoVision::getOptionsHordur(fovis::VisualOdometryOptions &vo_opts)
{
  // change to stereo 'defaults'
  vo_opts["use-adaptive-threshold"] = "false"; // hordur: use now not very useful - adds noisy features
  vo_opts["fast-threshold"] = "15";
  // hordur: use not and set fast-threshold as 10-15

  // options if uat is true
  vo_opts["feature-window-size"] = "9";
  vo_opts["max-pyramid-level"] = "3";
  vo_opts["min-pyramid-level"] = "0";
  vo_opts["target-pixels-per-feature"] = "250"; 
  //width*height/250 = target number of features for fast detector
  // - related to fast-threshold-adaptive-gain
  // 640x480 pr2 ---> 307200/tppf = nfeatures = 400 (typically for pr2)
  // 1024x620 (1088)> 634880/tppf = nfeatures
  vo_opts["fast-threshold-adaptive-gain"] = "0.002";
  vo_opts["use-homography-initialization"] = "true";
  vo_opts["ref-frame-change-threshold"] = "100"; // hordur: lowering this is a good idea. down to 100 is good. results in tracking to poses much further appart


  // OdometryFrame
  vo_opts["use-bucketing"] = "true"; // dependent on resolution: bucketing of features. might want to increase this...
  vo_opts["bucket-width"] = "50";
  vo_opts["bucket-height"] = "50";
  vo_opts["max-keypoints-per-bucket"] = "10";
  vo_opts["use-image-normalization"] = "true"; //hordur: not of major importance, can turn off, extra computation

  // MotionEstimator
  vo_opts["inlier-max-reprojection-error"] = "1.0"; // putting this down to 1.0 is good - give better alignment
  vo_opts["clique-inlier-threshold"] = "0.1";
  vo_opts["min-features-for-estimate"] = "10";
  vo_opts["max-mean-reprojection-error"] = "8.0";
  vo_opts["use-subpixel-refinement"] = "true"; // hordur: v.important to use
  vo_opts["feature-search-window"] = "25"; // for rapid motion this should be higher - size of area to search for new features
  vo_opts["update-target-features-with-refined"] = "false";

  // StereoDepth
  vo_opts["stereo-require-mutual-match"] = "true";
  vo_opts["stereo-max-dist-epipolar-line"] = "2.0";
  vo_opts["stereo-max-refinement-displacement"] = "2.0";
  vo_opts["stereo-max-disparity"] = "128";
}


void FoVision::getOptionsCommon(fovis::VisualOdometryOptions &vo_opts)
{
  // Commonly used settings for some time
  // 9.53fps on 0490-0515.lcmlog (from 2016-06-07 George Square log)

  // change to stereo 'defaults'
  vo_opts["use-adaptive-threshold"] = "true"; // hordur: use now not very useful - adds noisy features
  vo_opts["fast-threshold"] = "10";
  // hordur: use not and set fast-threshold as 10-15

  // options if uat is true
  vo_opts["feature-window-size"] = "9";
  vo_opts["max-pyramid-level"] = "3";
  vo_opts["min-pyramid-level"] = "0";
  vo_opts["target-pixels-per-feature"] = "250"; 
  //width*height/250 = target number of features for fast detector
  // - related to fast-threshold-adaptive-gain
  // 640x480 pr2 ---> 307200/tppf = nfeatures = 400 (typically for pr2)
  // 1024x620 (1088)> 634880/tppf = nfeatures
  vo_opts["fast-threshold-adaptive-gain"] = "0.002";
  vo_opts["use-homography-initialization"] = "true";
  vo_opts["ref-frame-change-threshold"] = "150"; // hordur: lowering this is a good idea. down to 100 is good. results in tracking to poses much further appart


  // OdometryFrame
  vo_opts["use-bucketing"] = "true"; // dependent on resolution: bucketing of features. might want to increase this...
  vo_opts["bucket-width"] = "50";
  vo_opts["bucket-height"] = "50";
  vo_opts["max-keypoints-per-bucket"] = "10";
  vo_opts["use-image-normalization"] = "true"; //hordur: not of major importance, can turn off, extra computation

  // MotionEstimator
  vo_opts["inlier-max-reprojection-error"] = "2.0"; // putting this down to 1.0 is good - give better alignment
  vo_opts["clique-inlier-threshold"] = "0.1";
  vo_opts["min-features-for-estimate"] = "10";
  vo_opts["max-mean-reprojection-error"] = "8.0";
  vo_opts["use-subpixel-refinement"] = "true"; // hordur: v.important to use
  vo_opts["feature-search-window"] = "25"; // for rapid motion this should be higher - size of area to search for new features
  vo_opts["update-target-features-with-refined"] = "false";

  // StereoDepth
  vo_opts["stereo-require-mutual-match"] = "true";
  vo_opts["stereo-max-dist-epipolar-line"] = "2.0";
  vo_opts["stereo-max-refinement-displacement"] = "2.0";
  vo_opts["stereo-max-disparity"] = "128";
}


void FoVision::getOptionsFaster(fovis::VisualOdometryOptions &vo_opts)
{
  // Modifications to specific VO parameters to achieve framerate
  // TODO: more fully evaluate the performance effect of this
  // 18.95fps on 0490-0515.lcmlog (from 2016-06-07 George Square log)

  vo_opts["target-pixels-per-feature"] = "400";

  vo_opts["bucket-width"] = "80";
  vo_opts["bucket-height"] = "80";
}
