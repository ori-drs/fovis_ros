//drc-descript-brief 0016_1349291130753495
// - read in 0016* files
// match to all others in that directory
//
// this was finished in jan 2012.
// next step is to create an application which uses this as part of an lcm stream

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <dirent.h>

#include <GL/gl.h>

#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"

#include <bot_lcmgl_client/lcmgl.h>


#include "vis_loop_closure/vis_loop_closure.hpp"


//#include <pronto_vis/pronto_vis.hpp>

using namespace std;
using namespace cv::xfeatures2d;



#include <chrono>
auto start = std::chrono::high_resolution_clock::now();
void tic(){
  start = std::chrono::high_resolution_clock::now();
}
void toc(std::string message){
  auto elapsed = std::chrono::high_resolution_clock::now() - start;
  double seconds_e = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count()*1E-6;
  std::cout << seconds_e << "sec " << message << "\n"; 
}



VisLoopClosure::VisLoopClosure(boost::shared_ptr<lcm::LCM> &lcm_, const VisLoopClosureConfig& reg_cfg_):
        lcm_(lcm_),reg_cfg_(reg_cfg_){
  std::string camera_config = "CAMERA";

  // Set up frames and config:
  if (reg_cfg_.param_file.empty()) {
    std::cout << "Get params from LCM\n";
    botparam_ = bot_param_get_global(lcm_->getUnderlyingLCM(), 0);
  } else {
    std::cout << "Get params from file\n";
    botparam_ = bot_param_new_from_file(reg_cfg_.param_file.c_str());
  }
  if (botparam_ == NULL) {
    exit(1);
  }
  botframes_= bot_frames_get_global(lcm_->getUnderlyingLCM(), botparam_);
  botframes_cpp_ = new bot::frames(botframes_);

  config_ = new voconfig::KmclConfiguration(botparam_, camera_config);
  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config_->load_stereo_calibration());

  fovis::CameraIntrinsicsParameters cip = stereo_calibration_->getRectifiedParameters();
  // fx 0  cx 0
  // 0  fy cy 0
  // 0  0  0  0
  projection_matrix_ << cip.fx,  0, cip.cx, 0,
                       0, cip.fy,  cip.cy, 0,
                       0,   0,   1, 0;

  // from newcollege_stereo config: (left)
  //projection_matrix_ << 389.956085,  0, 254.903519, 0,
  //                     0, 389.956085,  201.899490, 0,
  //                     0,   0,   1, 0;

  // bumblebee left:
  //projection_matrix_ << 836.466,  0, 513.198, 0,
  //                     0, 835.78,  397.901, 0,
  //                     0,   0,   1, 0;
      
  // loader multisense left:
  //projection_matrix_ << 606.0344848632812,  0, 512.0, 0,
  //                     0, 606.0344848632812,  272.0, 0,
  //                     0,   0,   1, 0;

  // husky unit 44
  // projection_matrix_ << 580.5900268554688,  0, 512.0, 0,
  //                     0, 580.5900268554688,  512.0, 0,
  //                     0,   0,   1, 0;

  imgutils_ = new image_io_utils( lcm_, stereo_calibration_->getWidth(), 2*stereo_calibration_->getHeight()); // extra space for stereo tasks

  int reset =1;
  // Vis Config:
  pc_vis_ = new pronto_vis(lcm_->getUnderlyingLCM());
  float colors_0f[] ={1.0,0.0,0.0};
  vector <float> colors_0;
  colors_0.assign(colors_0f,colors_0f+4*sizeof(float));
  float colors_1f[] ={0.0,0.0,1.0};
  vector <float> colors_1;
  colors_1.assign(colors_1f,colors_1f+4*sizeof(float));

  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(1000,"Pose A",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1002,"Cloud A"     ,1,reset, 1000,1,colors_0));
  colors_0[0] = 0.7;
  colors_0[1] = 0.7;
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(1001,"Cloud A - inliers"     ,1,reset, 1000,1,colors_0));

  pc_vis_->obj_cfg_list.push_back( obj_cfg(2000,"Pose B",5,0) );
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2002,"Cloud B"     ,1,reset, 2000,1,colors_1));
  colors_1[1] = 0.7;
  colors_1[2] = 0.7;
  pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(2001,"Cloud B - inliers"     ,1,reset, 2000,1,colors_1));

  features_ = new VoFeatures(lcm_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );

}


void features2cloud(std::vector<ImageFeature> features, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud){
  cloud->width   = features.size();
  cloud->height   = 1;
  cloud->points.resize (cloud->width  *cloud->height);
  for (size_t i=0;i<features.size(); i++){
    cloud->points[i].x = features[i].xyz[0];
    cloud->points[i].y = features[i].xyz[1];
    cloud->points[i].z = features[i].xyz[2];
    cloud->points[i].r = 1;
    cloud->points[i].g = 1;
    cloud->points[i].b = 1;
  }
}

// Send a subset of the feature set:
void features2cloud(std::vector<ImageFeature> features, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    std::vector<int> features_indices){
  cloud->width   = features_indices.size();
  cloud->height   = 1;
  cloud->points.resize (cloud->width  *cloud->height);
  for (size_t j=0;j<features_indices.size(); j++){
    int i = features_indices[j];
    cloud->points[j].x = features[i].xyz[0];
    cloud->points[j].y = features[i].xyz[1];
    cloud->points[j].z = features[i].xyz[2];
    cloud->points[j].r = 1;
    cloud->points[j].g = 1;
    cloud->points[j].b = 1;
  }
}



void VisLoopClosure::send_both_reg(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
    Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1,
    int64_t utime0, int64_t utime1           ){

  Eigen::Matrix3d M;
  M <<  0,  0, 1,
       -1,  0, 0,
        0, -1, 0;
  pose0 = M * pose0;
  pose1 = M * pose1;

  Isometry3dTime pose0T = Isometry3dTime(utime0, pose0);
  pc_vis_->pose_to_lcm_from_list(1000, pose0T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features0,cloud0);
  pc_vis_->ptcld_to_lcm_from_list(1002, *cloud0, utime0, utime0);

  Isometry3dTime pose1T = Isometry3dTime(utime1, pose1);
  pc_vis_->pose_to_lcm_from_list(2000, pose1T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features1,cloud1);
  pc_vis_->ptcld_to_lcm_from_list(2002, *cloud1, utime1, utime1);

}


void draw_inliers(cv::Mat &imgs, std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
    std::vector<int> feature_inliers0,    std::vector<int> feature_inliers1){

  cv::Point p;
  p.x =3; p.y = 10;
  CvScalar color_out = CV_RGB(255,0,0);
  for (size_t j=0;j< feature_inliers0.size(); j++){
    int i0 = feature_inliers0[j];
    cv::Point p0;
    p0.x = features0[i0].base_uv[0];
    p0.y = features0[i0].base_uv[1];
    cv::circle( imgs, p0, 5, color_out, 0 ); 

    int i1 = feature_inliers1[j];
    cv::Point p1;
    p1.x = features1[i1].base_uv[0] + imgs.cols/2; // offset by half the double image with
    p1.y = features1[i1].base_uv[1];
    cv::circle( imgs, p1, 5, color_out, 0 ); 
    cv::line(imgs, p0, p1, color_out, 2);
  }
}


void VisLoopClosure::send_both_reg_inliers(std::vector<ImageFeature> features0,    std::vector<ImageFeature> features1,
    Eigen::Isometry3d pose0,   Eigen::Isometry3d pose1,
    std::vector<int> feature_inliers0,    std::vector<int> feature_inliers1 ,
    int64_t utime0, int64_t utime1){

  Eigen::Matrix3d M;
  M <<  0,  0, 1,
       -1,  0, 0,
        0, -1, 0;
  pose0 = M * pose0;
  pose1 = M * pose1;

  Isometry3dTime pose0T = Isometry3dTime(utime0, pose0);
  pc_vis_->pose_to_lcm_from_list(1000, pose0T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud0 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features0,cloud0,feature_inliers0);
  pc_vis_->ptcld_to_lcm_from_list(1001, *cloud0, utime0, utime0);


  Isometry3dTime pose1T = Isometry3dTime(utime1, pose1);
  pc_vis_->pose_to_lcm_from_list(2000, pose1T);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGB> ());
  features2cloud(features1,cloud1,feature_inliers1);
  pc_vis_->ptcld_to_lcm_from_list(2001, *cloud1, utime1, utime1);

}








// 'lexicographic' comparison
static bool DMatch_lt(const cv::DMatch& a, const cv::DMatch& b) {
  //return ( (a.trainIdx < b.trainIdx) || (a.queryIdx < b.queryIdx) );
  if (a.trainIdx != b.trainIdx) { return a.trainIdx < b.trainIdx; }
  return a.queryIdx < b.queryIdx;
}

void features_to_keypoints(const std::vector<ImageFeature> & features, std::vector<cv::KeyPoint> & kpts) {
  kpts.clear();
  for(std::vector<ImageFeature>::const_iterator it=features.begin(); it != features.end(); ++it) {
    const ImageFeature & f = *it;
    // @todo use Mei's true scale here
    //    kpts.push_back(cv::KeyPoint(f.uvd(0), f.uvd(1), 1.0));
    kpts.push_back(cv::KeyPoint(f.base_uv(0), f.base_uv(1), 20.0)); // was in hordur's code
  }
}

void compute_descriptors(cv::Mat &image, vector<ImageFeature> & features, std::string & name, cv::DescriptorExtractor & extractor,
    cv::Mat &descriptors, std::vector<cv::KeyPoint>& keypoints){
  features_to_keypoints(features, keypoints);
  extractor.compute(image, keypoints, descriptors);
}


void VisLoopClosure::read_features(std::string fname,
    std::vector<ImageFeature>& features ){

  //printf( "About to read: %s - ",fname.c_str());
  int counter=0;
  string line0;
  std::ifstream myfile (fname.c_str());
  if (myfile.is_open()){

    getline (myfile,line0);
    //cout << line0 << " is first line\n";

    counter =0;
    while ( myfile.good() ){
      string line;
      getline (myfile,line);
      if (line.size() > 4){
        ImageFeature f;
        int i,track_id;
        double v[15];
        int d[3];
        int res = sscanf(line.c_str(), "%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d,%d",&i,
            &(v[0]), &(v[1]), &(v[2]), &(v[3]), &(v[4]), // id, uv, base_uv
            &(v[5]), &(v[6]), &(v[7]),  &(v[8]), &(v[9]), &(v[10]),// uvd xyz
            &(v[11]), &(v[12]), &(v[13]), &(v[14]), // xyzd
            &(d[0]), &(d[1]), &(d[2])        );

        f.track_id=v[0]; f.uv[0]=v[1];f.uv[1]=v[2];
        f.base_uv[0]=v[3];f.base_uv[1]=v[4];
        f.uvd[0]=v[5];f.uvd[1]=v[6];f.uvd[2]=v[7];
        f.xyz[0]=v[8];f.xyz[1]=v[9];f.xyz[2]=v[10];
        f.xyzw[0]=v[11];f.xyzw[1]=v[12];f.xyzw[2]=v[13];f.xyzw[3]=v[14];
        f.color[0] = d[0];f.color[1] = d[1];f.color[2] = d[2];

        /*
        cout << line << " is line\n";
        cout << "i: " << i <<"\n";
        cout << "f.track_id: " << f.track_id <<"\n";
        cout << "f.uv: " << f.uv[0] << " "<< f.uv[1] <<"\n";
        cout << "f.base_uv: " << f.base_uv[0] << " "<< f.base_uv[1] <<"\n";
        cout << "f.uvd: " << f.uvd[0] << " "<< f.uvd[1]<< " "<< f.uvd[2]<<"\n";
        cout << "f.xyz: " << f.xyz[0] << " "<< f.xyz[1]<< " "<< f.xyz[2]<<"\n";
        cout << "f.xyzw: " << f.xyzw[0] << " "<< f.xyzw[1]<< " "<< f.xyzw[2]<< " "<< f.xyzw[3]<<"\n";
        cout << "f.color: " << (int)f.color[0] << " "<< (int)f.color[1] << " "<< (int)f.color[2] <<"\n";
         */
        features.push_back(f);
      }
    }
    myfile.close();
  } else{
    printf( "Unable to open features file\n%s",fname.c_str());
    return;
  }
  //cout << "read " << features.size() << " features\n";
}


void pose_estimate(FrameMatchPtr match,
    std::vector<char> & inliers,
    Eigen::Isometry3d & motion,
    Eigen::MatrixXd & motion_covariance,
    Eigen::Matrix<double, 3, 4> & proj_matrix) {
  using namespace pose_estimator;

  PoseEstimator pe(proj_matrix);

  if ((match->featuresA_indices.size()!=match->featuresB_indices.size()))
    cout <<    "Number of features doesn't match\n";

  size_t num_matches = match->featuresA_indices.size();

  if(num_matches < 3)
    cout << "Need at least three matches to estimate pose";

  motion.setIdentity();
  motion_covariance.setIdentity();

  Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::ColMajor> src_xyzw(4, num_matches);
  Eigen::Matrix<double, 4, Eigen::Dynamic, Eigen::ColMajor>dst_xyzw(4, num_matches);
  for (size_t i=0; i < num_matches; ++i) {

    //    const ImageFeature& featureA(int i) const {
    //      assert (frameA);
    //    int ix = featuresA_indices.at(i);
    //      return frameA->features().at(ix);
    //    }
    //src_xyzw.col(i) = match->featureA(i).xyzw;
    //dst_xyzw.col(i) = match->featureB(i).xyzw;

    int ixA = match->featuresA_indices.at(i);
    int ixB = match->featuresB_indices.at(i);
    //cout << ixA << " | " << ixB << "\n";
    //cout << match->featuresA.size() << " fA size\n";
    //cout <<  match->featuresA[ixA].xyzw[0] << "\n";
    //cout <<  match->featuresA[ixA].xyzw[0] << "\n";

    src_xyzw.col(i) = match->featuresA[ixA].xyzw;
    dst_xyzw.col(i) = match->featuresB[ixB].xyzw;
  }

  // PoseEstimateStatus status = pe.estimate(src_xyzw, dst_xyzw, &inliers, &motion, &motion_covariance);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> motion_covariance_col_major;

  pose_estimator::PoseEstimateStatus status = pe.estimate(src_xyzw, dst_xyzw, &inliers, &motion, &motion_covariance_col_major); //&motion_covariance);
  motion_covariance = motion_covariance_col_major;

  match->status = status;
  match->delta = motion;

}



void VisLoopClosure::align_images(cv::Mat &img0, cv::Mat &img1, 
                           std::vector<ImageFeature> &features0, std::vector<ImageFeature> &features1,
                           int64_t utime0, int64_t utime1, FrameMatchPtr &match){

  // 1. Publish the original images with the features drawn on them
  if (reg_cfg_.publish_diagnostics){ 
    std::vector<int> fake_indices0(features0.size());
    for(int x = 0; x < features0.size(); ++x)
       fake_indices0[x] = 1;
    features_->sendImage("IMG0",img0.data, features0, fake_indices0);

    std::vector<int> fake_indices1(features1.size());
    for(int x = 0; x < features1.size(); ++x)
       fake_indices1[x] = 1;
    features_->sendImage("IMG1",img1.data, features1, fake_indices1);
  }

    /// 2. Extract Image Descriptors:
  //tic();
  std::vector<cv::KeyPoint> keypoints0, keypoints1;
  cv::Mat descriptors0, descriptors1;

  BriefDescriptorExtractor* extractor = BriefDescriptorExtractor::create(32);
  // opencv2:  cv::BriefDescriptorExtractor extractor(32); // size of descriptor in bytes

  std::string desc_name= "brief";
  compute_descriptors(img0, features0, desc_name, *extractor,descriptors0,keypoints0);
  compute_descriptors(img1, features1, desc_name, *extractor,descriptors1,keypoints1);
  //toc("compute_descriptors");

  if (reg_cfg_.verbose){
    cout << descriptors0.rows << " descripters in 0 | "
         << descriptors1.rows << " descripters in 1\n";
  }



  /// 3: Matching descriptor vectors with a brute force matcher
  //tic();
  cv::BFMatcher matcher(cv::NORM_HAMMING); // used by hordur
  std::vector< cv::DMatch > matches0in1,matches1in0;
  matcher.match(descriptors0, descriptors1, matches0in1); // each feature in 0 found in 1
  matcher.match(descriptors1, descriptors0, matches1in0); // each feature in 1 found in 0
  BOOST_FOREACH (cv::DMatch& match, matches1in0) {
    std::swap(match.trainIdx, match.queryIdx);
  }
  //toc("match descriptors");

  /// 3b keep intersection, aka the mutual best matches.
  //tic();
  std::sort(matches0in1.begin(), matches0in1.end(), DMatch_lt);
  std::sort(matches1in0.begin(), matches1in0.end(), DMatch_lt);
  std::vector<cv::DMatch> matches;
  std::set_intersection(matches0in1.begin(), matches0in1.end(),
      matches1in0.begin(), matches1in0.end(),
      std::back_inserter(matches),
      DMatch_lt);
  if (reg_cfg_.verbose){
    std::cout << matches0in1.size() << " matches0in1 | "
              << matches1in0.size() << " matches1in0 | "
              << matches.size() << " intersecting matches found\n";
  }

  /// 3c Draw descriptor matches
  cv::Mat img_matches_inter;
  if (reg_cfg_.publish_diagnostics || reg_cfg_.use_cv_show){
    cv::Mat img_matches0in1, img_matches1in0;
    cv::drawMatches( img0, keypoints0, img1, keypoints1, matches0in1, img_matches0in1 );
    //imshow("Matches0in1", img_matches0in1 );
    //cv::waitKey(0);  

    cv::drawMatches( img0, keypoints0, img1, keypoints1, matches1in0, img_matches1in0  );
    //imshow("Matches1in0 [matches1in0 reordered]", img_matches1in0 );
    //cv::waitKey(0); 
 
    cv::drawMatches( img0, keypoints0, img1, keypoints1, matches, img_matches_inter );
    //imshow("Matches Intersection", img_matches_inter );
  }
  
  /// 4 Pair up Matches: (TODO: understand this better)
  std::vector<int> idxA;
  std::vector<int> idxB;
  for (size_t i = 0; i < descriptors0.rows; ++i){
    //if (1==1){//(desc_A->valid[i]) {
    //descriptors0.row(i).copyTo(descriptors0.row(validA));
    idxA.push_back(i);
    //validA++;
    //}
  }
  for (size_t i = 0; i < descriptors1.rows; ++i){
    //if (desc_B->valid[i]) {
    //descriptorsB.row(validB) = desc_B->descriptors.row(i);
    //desc_B->descriptors.row(i).copyTo(descriptorsB.row(validB));
    idxB.push_back(i);
    //validB++;
    //}
  }

  std::vector<char> inliers;
  int num_inliers = 0;
  Eigen::Isometry3d motion;
  BOOST_FOREACH (const cv::DMatch& dmatch, matches) {
    //match->featuresA_indices.push_back(idxA[dmatch.queryIdx]);
    //match->featuresB_indices.push_back(idxB[dmatch.trainIdx]);
    match->featuresA_indices.push_back(dmatch.queryIdx);
    match->featuresB_indices.push_back(dmatch.trainIdx);
  }
  BOOST_FOREACH (const ImageFeature& feature, features0) {
    match->featuresA.push_back(feature);
  }
  BOOST_FOREACH (const ImageFeature& feature, features1) {
    match->featuresB.push_back(feature);
  }

  BOOST_FOREACH (cv::DMatch& dmatch, matches) {
    //dmatch.queryIdx = idxA[dmatch.queryIdx];
    //dmatch.trainIdx = idxB[dmatch.trainIdx];
  }
  //toc("sort descriptors");

  //tic();
  // 5 find the motion transform between to images
  if (matches.size() >= 3) {

    Eigen::MatrixXd covariance;
    pose_estimate(match, inliers, motion, covariance,
        projection_matrix_);


    size_t j = 0;
    for (size_t i = 0; i < inliers.size(); ++i){
      if (inliers[i]) {
        match->featuresA_indices[j] = match->featuresA_indices[i];
        match->featuresB_indices[j] = match->featuresB_indices[i];
        j++;
      }
    }

    match->featuresA_indices.resize(j);
    match->featuresB_indices.resize(j);
    match->n_inliers = inliers.size();//
    match->n_registration_inliers = std::accumulate(inliers.begin(), inliers.end(), 0);

  } else {
    // TODO: this should have a different code e.g. insufficient matches:
    match->status = pose_estimator::INSUFFICIENT_INLIERS;
    match->n_registration_inliers = 0;
  }

  

  if (match->n_registration_inliers >= reg_cfg_.min_inliers) {
    match->status = pose_estimator::SUCCESS;
  } else {
    match->status = pose_estimator::INSUFFICIENT_INLIERS;
  }


  // 6 Output diagnostics
  if( reg_cfg_.verbose){
    Eigen::Quaterniond delta_quat = Eigen::Quaterniond(match->delta.rotation());
    cout << match->delta.translation().x() << " "
      << match->delta.translation().y() << " "
      << match->delta.translation().z() << " | "
      << delta_quat.w() << " "
      << delta_quat.x() << " "
      << delta_quat.y() << " "
      << delta_quat.z() << " | inliers: "
      << match->n_registration_inliers << "\n";
  }

  if (reg_cfg_.publish_diagnostics || reg_cfg_.use_cv_show){
    if (match->status == pose_estimator::SUCCESS){
      // Draw the inlier set:
      draw_inliers(img_matches_inter,features0, features1,match->featuresA_indices, match->featuresB_indices);

      if (reg_cfg_.publish_diagnostics){
        Eigen::Isometry3d nullpose;
        nullpose.setIdentity();

        // all the features:
        if (reg_cfg_.publish_diagnostics)
          send_both_reg(features0, features1,match->delta,nullpose, utime0, utime1);
      
        // just the inliers:
        send_both_reg_inliers(features0, features1,nullpose,match->delta, 
                        match->featuresA_indices, match->featuresB_indices,
                        utime0, utime1);

        imgutils_->sendImageJpeg(img_matches_inter.data, 0, img_matches_inter.cols, img_matches_inter.rows, 94, "REGISTERATION", 3);
      }
    }
    if (reg_cfg_.use_cv_show){
      imshow("Matches Inliers", img_matches_inter);
      cv::waitKey(0);  
    }
  }
  //toc("align_images, rest");

}



// Get the ordered list of file names in the directory
// looking for files ending 'feat'
// TODO: should this be moved to image_database.cpp?
void VisLoopClosure::getFilenames(std::string path_to_folder, std::vector<std::string> &futimes, 
    std::vector<std::string> &utimes_strings){
  std::cout << "Reading files from " << path_to_folder << "\n";
  
  DIR *dir;
  struct dirent *ent;
  if ((dir = opendir (path_to_folder.c_str() )) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir (dir)) != NULL) {
      std::string fname = ent->d_name;
      if(fname.size() > 5){
        if (fname.compare(fname.size()-4,4,"feat") == 0){ 
          //printf ("%s\n", ent->d_name);
          futimes.push_back( fname.substr(0,21) );
        }
      }
    }
    closedir (dir);
  } else {
    /* could not open directory */
    perror ("");
    exit(-1);
  }

  std::sort(futimes.begin(), futimes.end());
  for (size_t i = 0; i<futimes.size(); i++){
    //std::cout << i << ": " << futimes[i] << "\n";
    utimes_strings.push_back(  futimes[i].substr(5,16) );
  }
  std::cout << "Found " << futimes.size() << " feature and image files\n";

}
