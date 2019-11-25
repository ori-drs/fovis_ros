#ifndef VOFEATURES_HPP_
#define VOFEATURES_HPP_

#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

#include <fovis/fovis.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "imagefeatures.hpp"

#include <pcl/io/io.h>


class VoFeatures
{
public:
  VoFeatures(int image_width_,
             int image_height_,
             const Eigen::Isometry3d& camera_to_body = Eigen::Isometry3d::Identity());

  // do nothing special, the buffers are set free by someone else
  virtual ~VoFeatures() = default;

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

  void drawFeaturesOnImage(uint8_t *img_buf,
                           std::vector<ImageFeature> &features,
                           std::vector<int> &feature_indices);

  void storeFeaturesAsCloud(std::vector<ImageFeature> features, 
                                std::vector<int> features_indices,
                                bool is_ref);

  // Returns the left image with the point features drawn on it
  uint8_t* getFeaturesImage(){ return left_cur_buf_; }

  // Returns a pcl point cloud of the points, in the robotics frame of the object
  pcl::PointCloud<pcl::PointXYZRGB> getFeaturesCloud(void){
    return features_cloud_;
  }


private:
  int image_width_;
  int image_height_;
  int output_counter_;
  
  void writeImage(uint8_t* img_buf, int counter, int64_t utime);
  void writeFeatures(std::vector<ImageFeature> features, int counter, int64_t utime);
  void writePose(Eigen::Isometry3d pose, int64_t utime);

  // All the incoming data and state:
  Eigen::Isometry3d ref_camera_pose_, cur_camera_pose_;
  Eigen::Isometry3d camera_to_body_;
  int64_t utime_;
  // pointers to reference image: (only used of visual output):
  uint8_t *left_ref_buf_;
  std::vector<ImageFeature> features_ref_;
  std::vector<int> features_ref_indices_; // 0 outlier, 1 inlier
  // pointers to reference image: (only used of visual output):
  uint8_t *left_cur_buf_;
  std::vector<ImageFeature> features_cur_;
  std::vector<int> features_cur_indices_;

  pcl::PointCloud<pcl::PointXYZRGB> features_cloud_;

  std::fstream output_pose_file_;
};

#endif
