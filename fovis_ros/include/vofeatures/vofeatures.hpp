#ifndef VOFEATURES_HPP_
#define VOFEATURES_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <lcm/lcm-cpp.hpp>
#include <fovis/fovis.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "imagefeatures.hpp"
//#include <pronto_vis/pronto_vis.hpp> // visualize pt clds

class VoFeatures
{
public:
  VoFeatures(int image_width_, int image_height_);
  ~VoFeatures();

  void setFeatures(const fovis::FeatureMatch* matches, int num_matches, int64_t utime);
  
  void setReferenceImage(uint8_t *left_ref_buf){    
    left_ref_buf_ = left_ref_buf; 
  }
  void setCurrentImage(uint8_t *left_cur_buf){    
    left_cur_buf_ = left_cur_buf; 
  }
  
  /*
  void setReferenceImages(uint8_t *left_ref_buf,uint8_t *right_ref_buf){    
    left_ref_buf_ = left_ref_buf; 
    right_ref_buf_ = right_ref_buf;   
  }
  void setCurrentImages(uint8_t *left_cur_buf,uint8_t *right_cur_buf){    
    left_cur_buf_ = left_cur_buf; 
    right_cur_buf_ = right_cur_buf;   
  }
  */
  
  void setReferenceCameraPose(Eigen::Isometry3d ref_camera_pose){    
    ref_camera_pose_ = ref_camera_pose;
  }
  void setCurrentCameraPose(Eigen::Isometry3d cur_camera_pose){    
    cur_camera_pose_ = cur_camera_pose;
  }

  void doFeatureProcessing(bool useCurrent, bool writeOutput = false);
  void sendImage(std::string channel, uint8_t *img_buf, std::vector<ImageFeature> &features,
                 std::vector<int> &feature_indices);

  //void getFeaturesFromLCM(const  reg::features_t* msg, std::vector<ImageFeature> &features, 
  //  Eigen::Isometry3d &pose);
  void sendFeaturesAsCollection(std::vector<ImageFeature> features, 
                                std::vector<int> features_indices,
                                int vs_id);

  uint8_t* getFeatureImage(){ return left_cur_buf_; }

private:
  //pronto_vis* pc_vis_;
  int image_width_;
  int image_height_;
  int output_counter_;
  
  void writeImage(uint8_t* img_buf, int counter, int64_t utime);
  void writeFeatures(std::vector<ImageFeature> features, int counter, int64_t utime);
  void writePose(Eigen::Isometry3d pose, int64_t utime);
  void sendFeatures(std::vector<ImageFeature> features, 
                    std::vector<int> features_indices, std::string channel,
                    Eigen::Isometry3d pose,
                    int64_t utime);

  void drawFeaturesOnImage(cv::Mat &img, std::vector<ImageFeature> &features,
                           std::vector<int> &feature_indices);

  // All the incoming data and state:
  Eigen::Isometry3d ref_camera_pose_, cur_camera_pose_;
  int64_t utime_;
  // pointers to reference image: (only used of visual output):
  uint8_t *left_ref_buf_;
  std::vector<ImageFeature> features_ref_;
  std::vector<int> features_ref_indices_; // 0 outlier, 1 inlier
  // pointers to reference image: (only used of visual output):
  uint8_t *left_cur_buf_;
  std::vector<ImageFeature> features_cur_;
  std::vector<int> features_cur_indices_;

  // no longer used:
  // uint8_t *right_ref_buf_, *right_cur_buf_;

  std::fstream output_pose_file_;
};

#endif
