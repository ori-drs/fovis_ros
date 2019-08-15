#include "vofeatures/vofeatures.hpp"
#include <string>
#include <iostream>
#include <fovision/common.hpp>

using namespace cv;

VoFeatures::VoFeatures(int image_width_, int image_height_):
  image_width_(image_width_), image_height_(image_height_), utime_(0), output_counter_(0)   {

  //if(!lcm_->good()){
  //  std::cerr <<"ERROR: lcm is not good()" <<std::endl;
  //}

  // Vis Config:
  // mfallon:
  /*
  pc_vis_ = new pronto_vis( lcm_->getUnderlyingLCM() );
  */
  float colors_g[] ={0.0,1.0,0.0};
  std::vector <float> colors_v_g;
  colors_v_g.assign(colors_g,colors_g+4*sizeof(float));
  float colors_b[] ={0.0,0.0,1.0};
  std::vector <float> colors_v_b;
  colors_v_b.assign(colors_b,colors_b+4*sizeof(float));
  int reset =1;
  
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  //pc_vis_->obj_cfg_list.push_back( obj_cfg(3000,"Reference Camera Pose",5,reset) );
  //pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3001,"Reference Features",1,1, 3000,1,colors_v_g));
  //pc_vis_->obj_cfg_list.push_back( obj_cfg(3002,"Current Camera Pose",5,reset) );
  //pc_vis_->ptcld_cfg_list.push_back( ptcld_cfg(3003,"Current Features",1,reset, 3002,1,colors_v_b));

  //imgutils_ = new image_io_utils( lcm_, image_width_, 2*image_height_); // extra space for stereo tasks

}

// TODO: remove fovis dependency entirely:
void VoFeatures::setFeatures(const fovis::FeatureMatch* matches, int num_matches, int64_t utime){
  utime_ = utime;

//  ////////////////// Features: ////////////////////////////////////////////
//  const fovis::MotionEstimator* motion = _odom->getMotionEstimator();
//  int num_matches = motion->getNumMatches();
//  const fovis::FeatureMatch* matches = motion->getMatches();

  features_ref_.clear();
  features_ref_indices_.clear();
  features_cur_.clear();
  features_cur_indices_.clear();

  // This gets features that were matched but not necessarily inliers.
  // TODO also get features that were not even matched, if it helps.
  int feature_index = 0;

  //cout << "num_matches: " << num_matches << "\n";
  for (int i=0; i < num_matches; ++i) {
    const fovis::FeatureMatch & m(matches[i]);
    // @todo move the border removal outside, because it depends on the feature descriptor.
    //       these limits are for the BRIEF descriptor.
    int margin = 30;

    // current frame is  Ref frame and fa/dA
    //RawFrame::ConstPtr raw_ref_frame = ref_frame->raw_frame();

    if (   m.ref_keypoint->base_uv[0] <= margin
        || m.ref_keypoint->base_uv[0] >= (image_width_-margin)
        || m.ref_keypoint->base_uv[1] <= margin
        || m.ref_keypoint->base_uv[1] >= (image_height_-margin)) continue;

    //if (   m.target_keypoint->base_uv[0] <= margin
    //    || m.target_keypoint->base_uv[0] >= (width-margin)
    //    || m.target_keypoint->base_uv[1] <= margin
    //    || m.target_keypoint->base_uv[1] >= (height-margin)) continue;

    ImageFeature fA, fB;
    float dA =m.ref_keypoint->disparity; // compute_disparity_(m.ref_keypoint->disparity);
    float dB = m.target_keypoint->disparity;
    //float dB = compute_disparity_(m.target_keypoint->disparity);

    // Workaround for the fovis DepthImage depth source
    // was: if (isnan(dA)) dA = compute_disparity_(m.ref_keypoint->xyz(2));
    // was: if (isnan(dB)) dB = compute_disparity_(m.target_keypoint->xyz(2));
    if (std::isnan(dA)) dA = m.ref_keypoint->xyz(2); // stereo disparity is 1-to-1
    if (std::isnan(dB)) dB = m.target_keypoint->xyz(2);

    //if (1==1){//was
    if(m.inlier){ // if the feature is an inlier - very important distinciton, mfallon
      fA.track_id = fB.track_id = m.track_id;

      /*
      bool use_refined = false;
      if (use_refined && m.status == fovis::MATCH_OK)
      {
        fA.xyz = m.ref_keypoint->xyz;
        fB.xyz = m.refined_target_keypoint.xyz;

        fA.xyzw = m.ref_keypoint->xyzw;
        fB.xyzw = m.refined_target_keypoint.xyzw;

        fA.uv = Eigen::Vector2d(m.ref_keypoint->kp.u, m.ref_keypoint->kp.v);
        fB.uv = Eigen::Vector2d(m.refined_target_keypoint.kp.u, m.refined_target_keypoint.kp.v);

        fA.base_uv = m.ref_keypoint->base_uv;
        fB.base_uv = m.refined_target_keypoint.base_uv;

        fA.uvd = Eigen::Vector3d(m.ref_keypoint->rect_base_uv[0], m.ref_keypoint->rect_base_uv[1], dA);
        fB.uvd = Eigen::Vector3d(m.refined_target_keypoint.rect_base_uv[0], m.refined_target_keypoint.rect_base_uv[1], dB);
      }
      else
      {*/
        fA.xyz = m.ref_keypoint->xyz;
        fA.xyzw = m.ref_keypoint->xyzw;
        fA.uv = Eigen::Vector2d(m.ref_keypoint->kp.u, m.ref_keypoint->kp.v);
        fA.base_uv = m.ref_keypoint->base_uv;
        fA.uvd = Eigen::Vector3d(m.ref_keypoint->rect_base_uv[0], m.ref_keypoint->rect_base_uv[1], dA);

        fB.xyz = m.target_keypoint->xyz;
        fB.xyzw = m.target_keypoint->xyzw;
        fB.uv = Eigen::Vector2d(m.target_keypoint->kp.u, m.target_keypoint->kp.v);
        fB.base_uv = m.target_keypoint->base_uv;
        fB.uvd = Eigen::Vector3d(m.target_keypoint->rect_base_uv[0], m.target_keypoint->rect_base_uv[1], dB);
      //}
      features_ref_.push_back(fA);
      features_cur_.push_back(fB);

      if (m.inlier) {
        features_ref_indices_.push_back(1);
        features_cur_indices_.push_back(1);
      }else{
        features_ref_indices_.push_back(0);
        features_cur_indices_.push_back(0);
      }
      feature_index++;
    }
  }
}



// reference_or_current = 0 send reference (at the change of a key frame change)
// reference_or_current = 1 send current (otherwise)
void VoFeatures::doFeatureProcessing(bool useCurrent, bool writeOutput){
  if (writeOutput){
    if (!useCurrent){
    //  std::cout << "write reference features\n";
    //  writeImage(left_ref_buf_, output_counter_, utime_);
    //  writeFeatures(features_ref_, output_counter_, utime_);
    //  writePose(ref_camera_pose_, utime_);
    }else{
      std::cout << "write current features\n";
      writeImage(left_cur_buf_, output_counter_, utime_);
      writeFeatures(features_cur_, output_counter_, utime_);
      writePose(cur_camera_pose_, utime_);
    }
  }

  
  if(!useCurrent){ // reference
    // This timestamp is incorrect. need to get and cache the correct one:
    sendFeatures(features_ref_,features_ref_indices_, "FEATURES_REF", ref_camera_pose_, utime_);
    sendImage("REF_LEFT",left_ref_buf_, features_ref_, features_ref_indices_);
    //Isometry3dTime ref_camera_poseT = Isometry3dTime(utime_, ref_camera_pose_);
  
    // Send Features paired with the pose at the ref frame:
    //pc_vis_->pose_to_lcm_from_list(3000, ref_camera_poseT);
    sendFeaturesAsCollection(features_ref_, features_ref_indices_, 3001); // red, most recent 

    //pc_vis_->pose_to_lcm_from_list(3002, ref_camera_poseT);
    //sendFeaturesAsCollection(features_ref_, features_ref_indices_, 3003); // blue, last 
  }else{ // current
    sendFeatures(features_cur_,features_cur_indices_,"FEATURES_CUR", cur_camera_pose_, utime_);
    sendImage("CUR_LEFT",left_cur_buf_, features_cur_, features_cur_indices_);
    //Isometry3dTime cur_camera_poseT = Isometry3dTime(utime_, cur_camera_pose_);

    // Send Features paired with the pose at the ref frame:
    //pc_vis_->pose_to_lcm_from_list(3000, cur_camera_poseT);
    //sendFeaturesAsCollection(features_cur_, features_cur_indices_, 3001); // red, most recent 

    //pc_vis_->pose_to_lcm_from_list(3002, cur_camera_poseT);
    sendFeaturesAsCollection(features_cur_, features_cur_indices_, 3003); // blue, last 
  }

  
  output_counter_++;
}


void VoFeatures::drawFeaturesOnImage(cv::Mat &img, std::vector<ImageFeature> &features,
    std::vector<int> &feature_indices){
  CvScalar color_out = CV_RGB(255,255,255);
  for (size_t j=0;j< features.size(); j++){
    if (feature_indices[j]){
      cv::Point p0;
      p0.x = features[j].base_uv[0]; 
      p0.y = features[j].base_uv[1];
      cv::circle( img, p0, 5, color_out, 0 ); 
    }
  }  
}

void VoFeatures::sendImage(std::string channel, uint8_t* img_buf, std::vector<ImageFeature> &features,
    std::vector<int> &feature_indices){
  Mat img = Mat::zeros( image_height_, image_width_,CV_8UC1);
  img.data = img_buf;

  drawFeaturesOnImage(img, features, feature_indices);
  //REPLACE imgutils_->sendImageJpeg(img.data, 0, img.cols, img.rows, 94, channel, 1);
}

void VoFeatures::publishImage(std::string channel, cv::Mat img, int n_colors){
  /*
  bot_core_image_t image;
  image.utime =0;
  image.width = img.cols;
  image.height= img.rows;
  image.row_stride =n_colors*img.cols;
  image.pixelformat =BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY;
  image.size =n_colors*img.cols*img.rows;
  image.data = img.data;
  image.nmetadata =0;
  image.metadata = NULL;
  bot_core_image_t_publish( lcm_->getUnderlyingLCM() , channel.c_str(), &image);    
  */
}


void VoFeatures::writeImage(uint8_t* img_buf, int counter, int64_t utime){
  //cout << "images written to file @ " << utime_ << "\n";
  std::stringstream ss;
  char buff[10];
  sprintf(buff,"%.4d",counter);
  ss << buff << "_" << utime;
  Mat img = Mat::zeros( image_height_, image_width_,CV_8UC1);
  img.data = img_buf;
  imwrite( ( ss.str() + "_left.png"), img);
}

void VoFeatures::writeFeatures(std::vector<ImageFeature> features, int counter, int64_t utime){
  std::stringstream ss;
  char buff[10];
  sprintf(buff,"%.4d",counter);
  ss << buff << "_" << utime;

  std::fstream feat_file;
  std::string fname = std::string(  ss.str() + ".feat");
  feat_file.open(  fname.c_str() , std::fstream::out);
//  cout << "nmatches written to file: "<< features.size() << " @ " << utime_ << "\n";
  feat_file << "#i,track_id,uv,base_uv,uvd,xyz,xyzw,color\n";
  for (size_t i = 0; i < features.size(); i++) {
    ImageFeature f = features[i];
    std::ostringstream temp2;
    temp2 << i << ","
        << f.track_id << ","
        << f.uv[0] << "," << f.uv[1] << "," // actual pixel locations of features
        << f.base_uv[0] << "," << f.base_uv[1] << ","
        << f.uvd[0] << "," << f.uvd[1] << "," << f.uvd[2] << ","
        << f.xyz[0] << "," << f.xyz[1] << "," << f.xyz[2] << ","
        << f.xyzw[0] << "," << f.xyzw[1] << "," << f.xyzw[2] << "," << f.xyzw[3] << ","
        << (int) f.color[0] << "," << (int) f.color[1] << "," << (int) f.color[2] << "\n";
    feat_file << temp2.str() ;
  }
  feat_file.close();
}


void VoFeatures::writePose(Eigen::Isometry3d pose, int64_t utime){

  if(!output_pose_file_.is_open()){
    std::cout << "camera_trajectory.txt file not open, opening it\n";
    output_pose_file_.open(  "camera_trajectory.txt" , std::fstream::out);
    output_pose_file_ << "# utime x y z qw qx qy qz roll pitch yaw\n";
    output_pose_file_.flush();
  }

  double pose_rpy[3];
  Eigen::Quaterniond pose_quat = Eigen::Quaterniond( pose.rotation() );
  quat_to_euler( pose_quat, pose_rpy[0], pose_rpy[1], pose_rpy[2]);

  if(output_pose_file_.is_open()){
    output_pose_file_ << utime  << ","
                    << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
                    << pose_quat.w() << "," << pose_quat.x() << "," << pose_quat.y() << "," << pose_quat.z() << ","
                    << pose_rpy[0] << "," << pose_rpy[1] << "," << pose_rpy[2] << "\n";
    output_pose_file_.flush();                   
  }else{
    std::cout << "file not open still\n";
  }

}

void VoFeatures::sendFeaturesAsCollection(std::vector<ImageFeature> features, 
                                          std::vector<int> features_indices,
                                          int vs_id){
  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
  for (size_t i = 0; i < features.size(); i++) {
    if (features_indices[i]){
      ImageFeature f = features[i];
      pcl::PointXYZRGB pt;
      pt.x = f.xyz[0];
      pt.y = f.xyz[1];
      pt.z = f.xyz[2];
      pt.r =0; pt.g =0; pt.b =255.0; // blue
      cloud->points.push_back(pt);
    }
  }
  */
  //pc_vis_->ptcld_to_lcm_from_list(vs_id, *cloud, utime_, utime_);
}

void VoFeatures::sendFeatures(std::vector<ImageFeature> features, 
                              std::vector<int> features_indices, 
                              std::string channel,
                              Eigen::Isometry3d pose,
                              int64_t utime){
  
  /*
  reg::features_t msg;
  msg.utime = utime;
  for (size_t i = 0; i < features.size(); i++) {
    if (features_indices[i]){
      reg::feature_t* feat(new reg::feature_t());
      ImageFeature f = features[i];
      feat->track_id = f.track_id;
      feat->uv[0] = f.uv[0];
      feat->uv[1] = f.uv[1];
      feat->base_uv[0] = f.base_uv[0];
      feat->base_uv[1] = f.base_uv[1];
      feat->uvd[0] = f.uvd[0];
      feat->uvd[1] = f.uvd[1];
      feat->uvd[2] = f.uvd[2];
      feat->xyz[0] = f.xyz[0];
      feat->xyz[1] = f.xyz[1];
      feat->xyz[2] = f.xyz[2];
      feat->xyzw[0] = f.xyzw[0];
      feat->xyzw[1] = f.xyzw[1];
      feat->xyzw[2] = f.xyzw[2];
      feat->xyzw[3] = f.xyzw[3];
      // color left out for now
      msg.features.push_back(*feat);
    }
  }
  msg.nfeatures= msg.features.size();

  Eigen::Vector3d t(pose.translation());
  Eigen::Quaterniond r(pose.rotation());
  msg.pos[0] = t[0];
  msg.pos[1] = t[1];
  msg.pos[2] = t[2];
  msg.quat[0] = r.w();
  msg.quat[1] = r.x();
  msg.quat[2] = r.y();
  msg.quat[3] = r.z();
  lcm_->publish(channel.c_str(), &msg);
  */
}


/*
void VoFeatures::getFeaturesFromLCM(const  reg::features_t* msg, std::vector<ImageFeature> &features, Eigen::Isometry3d &pose){
  //std::vector<ImageFeature> features;
    
  for (int i =0; i < msg->nfeatures; i++){ 
    ImageFeature f;
    
    f.track_id = msg->features[i].track_id;
    f.uv[0] = msg->features[i].uv[0];
    f.uv[1] = msg->features[i].uv[1];
    f.base_uv[0] = msg->features[i].base_uv[0];
    f.base_uv[1] = msg->features[i].base_uv[1];
    f.uvd[0] = msg->features[i].uvd[0];
    f.uvd[1] = msg->features[i].uvd[1];
    f.uvd[2] = msg->features[i].uvd[2];
    f.xyz[0] = msg->features[i].xyz[0];
    f.xyz[1] = msg->features[i].xyz[1];
    f.xyz[2] = msg->features[i].xyz[2];
    f.xyzw[0] = msg->features[i].xyzw[0];
    f.xyzw[1] = msg->features[i].xyzw[1];
    f.xyzw[2] = msg->features[i].xyzw[2];
    f.xyzw[3] = msg->features[i].xyzw[3];
    // color left out for now
    
    pose.setIdentity();
    pose.translation() << msg->pos[0], msg->pos[1], msg->pos[2];
    Eigen::Quaterniond m(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
    pose.rotate(m);
    cout << line << " is line\n";
    cout << "i: " << i <<"\n";
    cout << "f.track_id: " << f.track_id <<"\n";
    cout << "f.uv: " << f.uv[0] << " "<< f.uv[1] <<"\n";
    cout << "f.base_uv: " << f.base_uv[0] << " "<< f.base_uv[1] <<"\n";
    cout << "f.uvd: " << f.uvd[0] << " "<< f.uvd[1]<< " "<< f.uvd[2]<<"\n";
    cout << "f.xyz: " << f.xyz[0] << " "<< f.xyz[1]<< " "<< f.xyz[2]<<"\n";
    cout << "f.xyzw: " << f.xyzw[0] << " "<< f.xyzw[1]<< " "<< f.xyzw[2]<< " "<< f.xyzw[3]<<"\n";
    cout << "f.color: " << (int)f.color[0] << " "<< (int)f.color[1] << " "<< (int)f.color[2] <<"\n";

    features.push_back(f);
  }
  
  //cout << "in: " << msg->nfeatures << " | extracted: "<< features.size() << "\n"; 
  return;  
}
*/
