// simple test program:
// se-simple-vo -L ~/drc-testing-data/state_est/run1_field_camera_snippet.lcmlog -P drc_robot.cfg -p

// Just do VO, not do any imu integration, dont correct for any urdf/cfg offsets
// Output motion in x forward, z up frame

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
//#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <lcmtypes/bot_core.hpp>

#include "voconfig/voconfig.hpp"
#include "vofeatures/vofeatures.hpp"
#include "voestimator/voestimator.hpp"
#include "fovision/fovision.hpp"

#include <pronto_vis/pronto_vis.hpp> // visualize pt clds
#include <pronto_conversions/pronto_conversions_lcm.hpp> // visualize pt clds
#include <ConciseArgs>

#include <path_util/path_util.h>
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression

#include <opencv/cv.h> // for disparity 

using namespace std;
using namespace cv; // for disparity ops

struct CommandLineConfig
{
  std::string camera_config; // which block from the cfg to read
  int fusion_mode;
  std::string output_extension;
  bool output_signal;
  std::string body_channel;
  bool vicon_init; // initialize using vicon/rigid_transform_t
  std::string vicon_init_channel;
  bool pose_init; // initialize using pose_t
  std::string pose_init_channel;
  std::string input_channel;
  bool verbose;
  std::string in_log_fname;
  std::string param_file;
  bool draw_lcmgl;
};



int get_trans_with_utime(BotFrames *bot_frames,
        const char *from_frame, const char *to_frame, int64_t utime,
        Eigen::Isometry3d & mat){
  int status;
  double matx[16];
  status = bot_frames_get_trans_mat_4x4_with_utime( bot_frames, from_frame,  to_frame, utime, matx);
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      mat(i,j) = matx[i*4+j];
    }
  }  

  return status;
}

class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_);
    
    ~StereoOdom(){
      free (left_buf_);
      free(right_buf_);
    }

  private:
    const CommandLineConfig cl_cfg_;    
    
    int image_size_; // just the resolution of the image
    uint8_t* left_buf_;
    uint8_t* right_buf_;
    mutable std::vector<float> disparity_buf_; // Is mutable necessary?
    uint8_t* rgb_buf_ ;
    uint8_t* decompress_disparity_buf_;    
    image_io_utils*  imgutils_;    
    
    int64_t utime_cur_;

    boost::shared_ptr<lcm::LCM> lcm_recv_;
    boost::shared_ptr<lcm::LCM> lcm_pub_;
    BotParam* botparam_;
    BotFrames* botframes_;
    //bot::frames* botframes_cpp_;
    voconfig::KmclConfiguration* config;
    pronto_vis* pc_vis_;

    //
    FoVision* vo_;

    //
    VoFeatures* features_;
    int64_t utime_prev_;
    uint8_t* left_buf_ref_; // copies of the reference images - probably can be extracted from fovis directly
    int64_t ref_utime_;
    Eigen::Isometry3d ref_camera_pose_; // pose of the camera when the reference frames changed
    bool changed_ref_frames_;

    int64_t deltaroot_utime_;
    Eigen::Isometry3d deltaroot_body_pose_; // pose of the body when the reference frames changed
    int deltaroot_counter_; // Setting to -1 will reset the deltaroot during the next iteration
    int frame_counter_; // counter of ALL received frames

    VoEstimator* estimator_;

    void featureAnalysis();
    void updateMotion(int64_t utime, int64_t prev_utime);

    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);

    void viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg);
    void poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);

    void deltaResetHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg);

    void initialiseCameraPose(Eigen::Isometry3d world_to_body_init, int64_t utime);

    // Filter the disparity image
    void filterDisparity(const  bot_core::images_t* msg, int w, int h);
    // Republish image to LCM. Used to examine the disparity filtering
    void republishImage(const  bot_core::images_t* msg);

    bool pose_initialized_;
    
    Eigen::Isometry3d world_to_camera_;
    Eigen::Isometry3d world_to_body_;


    // Image prefiltering
    bool filter_disparity_;
    double filter_disparity_below_threshold_;
    double filter_disparity_above_threshold_;
    int filter_image_rows_above_;
    bool publish_filtered_image_;

    // Settings
    bool publish_feature_analysis_;
    int feature_analysis_publish_period_; // number of frames between publishing the point features 
    bool check_valid_camera_frame_;

    int deltaroot_skip_;
    std::string deltaroot_reset_channel_;

    int process_skip_;
};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_) : 
       lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_), cl_cfg_(cl_cfg_), utime_cur_(0), utime_prev_(0), 
       ref_utime_(0), changed_ref_frames_(false)
{
  if (cl_cfg_.param_file.empty()) {
    std::cout << "Get param from LCM\n";
    botparam_ = bot_param_get_global(lcm_recv_->getUnderlyingLCM(), 0);
  } else {
    std::cout << "Get param from file\n";
    botparam_ = bot_param_new_from_file(cl_cfg_.param_file.c_str());
  }
  if (botparam_ == NULL) {
    exit(1);
  }
  botframes_ = bot_frames_get_global(lcm_recv_->getUnderlyingLCM(), botparam_);
  //botframes_cpp_ = new bot::frames(botframes_);

  // Disparity filtering
  filter_disparity_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.filter.enabled");
  std::cout << "Disparity Filter is " << (filter_disparity_ ? "ENABLED" : "DISABLED")  << "\n";
  filter_disparity_below_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_disparity_below_threshold");
  filter_disparity_above_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_disparity_above_threshold");
  filter_image_rows_above_ = bot_param_get_int_or_fail(botparam_, "visual_odometry.filter.filter_image_rows_above");
  publish_filtered_image_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.filter.publish_filtered_image");

  // Feature publishing (debug only)
  publish_feature_analysis_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.publish_feature_analysis");
  feature_analysis_publish_period_ = bot_param_get_int_or_fail(botparam_, "visual_odometry.feature_analysis_publish_period");

  check_valid_camera_frame_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.check_valid_camera_frame");
  process_skip_ = bot_param_get_int_or_fail(botparam_, "visual_odometry.process_skip");

  deltaroot_skip_ = bot_param_get_int_or_fail(botparam_, "visual_odometry.deltaroot_skip");
  char * deltaroot_reset_channel_char = bot_param_get_str_or_fail(botparam_, "visual_odometry.deltaroot_reset_channel");
  deltaroot_reset_channel_ = std::string(deltaroot_reset_channel_char);
  free(deltaroot_reset_channel_char);


  pc_vis_ = new pronto_vis( lcm_pub_->getUnderlyingLCM() );
  int reset =1;
  // obj: id name type reset
  // pts: id name type reset objcoll usergb rgb
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3004,"Delta Root Body Pose",5,reset) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3005,"Current Body Pose",5,reset) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3006,"Relative Body Motion",5,reset) );

  pc_vis_->obj_cfg_list.push_back( obj_cfg(3007,"VO Body Motion since ref",5,reset) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3008,"VO Body Motion original",5,reset) );
  pc_vis_->obj_cfg_list.push_back( obj_cfg(3009,"VO body alt",5,reset) );
  
  // Read config from file:
  config = new voconfig::KmclConfiguration(botparam_, cl_cfg_.camera_config);

  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config->load_stereo_calibration());
  image_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  left_buf_ = (uint8_t*) malloc(3*image_size_);
  right_buf_ = (uint8_t*) malloc(3*image_size_);
  left_buf_ref_ = (uint8_t*) malloc(3*image_size_); // used of feature output 
  rgb_buf_ = (uint8_t*) malloc(10*image_size_ * sizeof(uint8_t)); 
  decompress_disparity_buf_ = (uint8_t*) malloc( 4*image_size_*sizeof(uint8_t));  // arbitary size chosen..
  

  int which_vo_options = bot_param_get_int_or_fail(botparam_, "visual_odometry.which_vo_options");
  vo_ = new FoVision(lcm_pub_ , stereo_calibration_, cl_cfg_.draw_lcmgl, which_vo_options);
  vo_->setPublishFovisStats(publish_feature_analysis_);
  features_ = new VoFeatures(lcm_pub_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );
  estimator_ = new VoEstimator(lcm_pub_ , botframes_, cl_cfg_.output_extension, cl_cfg_.camera_config );
  lcm_recv_->subscribe( cl_cfg_.input_channel,&StereoOdom::multisenseHandler,this);

  pose_initialized_ = false;
  if (cl_cfg_.vicon_init){
    std::cout << "Will Init internal est using "  << cl_cfg_.vicon_init_channel << " message [vicon/rigid_trans]\n";
    lcm_recv_->subscribe(cl_cfg_.vicon_init_channel, &StereoOdom::viconHandler,this);
  }else if(cl_cfg_.pose_init){
    std::cout << "Will Init internal est using " << cl_cfg_.pose_init_channel << " message [pose]\n";
    lcm_recv_->subscribe(cl_cfg_.pose_init_channel, &StereoOdom::poseHandler,this);
  }else{
    std::cout << "Init internal est using default pose\n";

    // Useful for Atlas logs: initialise with nominal camera frame with the head pointing horizontally
    Eigen::Matrix3d M;
    M <<  0,  0, 1,
        -1,  0, 0,
          0, -1, 0;
    world_to_camera_ = M * Eigen::Isometry3d::Identity();
    world_to_camera_.translation().x() = 0;
    world_to_camera_.translation().y() = 0;
    world_to_camera_.translation().z() = 0;

    // Useful for Valkyrie logs: initialise with camera frame upside down but horizontal
    /*Eigen::Matrix3d M;
    M <<  0,  0, 1,
        1,  0, 0,
          0, 1, 0;
    world_to_camera_ = M * Eigen::Isometry3d::Identity();
    world_to_camera_.translation().x() = 0;
    world_to_camera_.translation().y() = 0;
    world_to_camera_.translation().z() = 1.65; // nominal head height
    */

    pose_initialized_ = true;
  }
  
  lcm_recv_->subscribe( deltaroot_reset_channel_ ,&StereoOdom::deltaResetHandler,this);
  deltaroot_counter_ = 0;

  frame_counter_ = 0;
  imgutils_ = new image_io_utils( lcm_pub_, stereo_calibration_->getWidth(), 2*stereo_calibration_->getHeight()); // extra space for stereo tasks


  cout <<"StereoOdom Constructed\n";
}


int counter =0;
void StereoOdom::featureAnalysis(){

  /// Incremental Feature Output:
  if (counter% feature_analysis_publish_period_  == 0 ){
    features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , utime_cur_);
    features_->setCurrentImage(left_buf_);
    //features_->setCurrentImages(left_buf_, right_buf_);
    //features_->setCurrentCameraPose( estimator_->getCameraPose() );
    features_->setCurrentCameraPose( world_to_camera_ );
    features_->doFeatureProcessing(1); // 1 = send the FEATURES_CUR

    Isometry3dTime cur_body_poseT = Isometry3dTime(utime_cur_, world_to_body_);
    pc_vis_->pose_to_lcm_from_list(3005, cur_body_poseT);

  }
  
  /// Reference Feature Output: ///////////////////////////////////////////////
  // Check we changed reference frame last iteration, if so output the set of matching inliers:
  if (changed_ref_frames_) {
    if (ref_utime_ > 0){ // skip the first null image
      if(vo_->getNumMatches() > 200){ // if less than 50 features - dont bother writing
      // was:      if(featuresA.size() > 50){ // if less than 50 features - dont bother writing
        std::cout.precision(17);
        cout << "VO keyframe changed from " << utime_prev_*1E-6 << " at " << utime_cur_*1E-6 <<  " with " <<vo_->getNumMatches()<<" matches\n";
        std::cout.precision(6);
        features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , ref_utime_);
        features_->setReferenceImage(left_buf_ref_);
        features_->setReferenceCameraPose( ref_camera_pose_ );
        features_->doFeatureProcessing(0); // 0 = send the FEATURES_REF
      }
    }
    changed_ref_frames_=false;
  }

  
  if (vo_->getChangeReferenceFrames()){ // If we change reference frame, note the change for the next iteration.
    //std::cout << "ref frame from " << ref_utime_ << " to " << utime_cur_  << " " << (utime_cur_-ref_utime_)*1E-6 << "sec\n";
    //ref_utime_ = utime_cur_;
    ref_camera_pose_ = world_to_camera_;
    ref_utime_ = utime_cur_;
    //deltaroot_body_pose_ = world_to_body_;
    // TODO: only copy gray data if its grey
    std::copy( left_buf_ , left_buf_ + 3*image_size_  , left_buf_ref_); // Keep the image buffer to draw the features in
    changed_ref_frames_=true;
  }
  
  counter++;
}



BotTrans getPoseAsBotTrans(Eigen::Isometry3d odo_delta){
  BotTrans msgT;
  memset(&msgT, 0, sizeof(msgT));
  Eigen::Vector3d motion_T = odo_delta.translation();
  Eigen::Quaterniond motion_R = Eigen::Quaterniond(odo_delta.rotation());
  msgT.trans_vec[0] = motion_T(0);
  msgT.trans_vec[1] = motion_T(1);
  msgT.trans_vec[2] = motion_T(2);
  msgT.rot_quat[0] = motion_R.w();
  msgT.rot_quat[1] = motion_R.x();
  msgT.rot_quat[2] = motion_R.y();
  msgT.rot_quat[3] = motion_R.z();
  
  return msgT;
}

Eigen::Isometry3d getBotTransAsPose(BotTrans msgT){
  Eigen::Isometry3d iso;
  iso.setIdentity();
  iso.translation().x() = msgT.trans_vec[0];
  iso.translation().y() = msgT.trans_vec[1];
  iso.translation().z() = msgT.trans_vec[2];
  Eigen::Quaterniond motion_R = Eigen::Quaterniond(msgT.rot_quat[0], msgT.rot_quat[1], msgT.rot_quat[2], msgT.rot_quat[3]);
  iso.rotate( motion_R );

  return iso;
}


Eigen::Isometry3d Isometry_invert_and_compose(Eigen::Isometry3d curr, Eigen::Isometry3d prev){
  // the typical Isometry3d transformation I have used isn't stable or doesnt work as envisaged...
  // Eigen::Isometry3d delta_body_from_ref =  world_to_body_ * ( deltaroot_body_pose_.inverse() );
  // TODO: really need to solve this, perhaps with Affine instead
  // In the meantime the BotTrans calculations used here are consistent with libbot

  BotTrans curr_BT = getPoseAsBotTrans(curr);
  BotTrans prev_BT = getPoseAsBotTrans(prev);  
  BotTrans diff_BT;
  memset(&diff_BT, 0, sizeof(diff_BT));

  bot_trans_invert_and_compose(&curr_BT, &prev_BT, &diff_BT);
  Eigen::Isometry3d diff = getBotTransAsPose(diff_BT);
  return diff;
}


void StereoOdom::updateMotion(int64_t utime, int64_t prev_utime){

  // 0. Reset the deltaroot. This can happen either after a set number of frames
  // or after a failure of the VO algorithm
  deltaroot_counter_++;
  if(deltaroot_counter_%deltaroot_skip_ == 0){
    std::cout.precision(17);
    std::cout << "Delta root frame changed from " << (deltaroot_utime_*1E-6) << " to " << (utime_cur_*1E-6)  << " " << (utime_cur_-deltaroot_utime_)*1E-6 << "sec\n";
    std::cout.precision(6);
    deltaroot_utime_ = utime_cur_;
    deltaroot_body_pose_ = world_to_body_;
  }

  // Update the camera position in world frame
  Eigen::Isometry3d delta_camera;
  Eigen::MatrixXd delta_camera_cov;
  fovis::MotionEstimateStatusCode delta_status;
  vo_->getMotion(delta_camera, delta_camera_cov, delta_status );
  vo_->fovis_stats();
  world_to_camera_  = world_to_camera_ * delta_camera;
  
  // 1 Determine the body position in world frame using the camera frame
  Eigen::Isometry3d camera_to_body;
  int status = get_trans_with_utime( botframes_ ,  "body", string( cl_cfg_.camera_config + "_LEFT" ).c_str()  , utime, camera_to_body);
  Eigen::Isometry3d new_world_to_body = world_to_camera_ * camera_to_body;  

  // 2 Find the delta in body motion by comparing the body position estimate with its previous
  //Eigen::Isometry3d delta_body =  new_world_to_body * ( .inverse() );
  Eigen::Isometry3d delta_body = Isometry_invert_and_compose(new_world_to_body, world_to_body_);
  world_to_body_ = new_world_to_body;

  // 3 Get relative transform since last keyframe change:
  //Eigen::Isometry3d delta_body_from_ref =  world_to_body_ * ( deltaroot_body_pose_.inverse() );
  Eigen::Isometry3d delta_body_from_ref = Isometry_invert_and_compose(world_to_body_, deltaroot_body_pose_);

  // 4. Output some signals
  if (cl_cfg_.output_signal ){
    Eigen::Isometry3d vel_body = pronto::getDeltaAsVelocity(delta_body, (utime-prev_utime) );

    estimator_->publishPose(utime, "POSE_CAMERA_LEFT_ALT", world_to_camera_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    estimator_->publishPose(utime, cl_cfg_.body_channel, world_to_body_, vel_body.translation(), Eigen::Vector3d::Zero());
  }

  // 4a. Exist of the most recent VO frame was a failure.
  if (delta_status != fovis::SUCCESS){
    std::cout << "Not sending delta travelled, resetting deltaroot\n";
    deltaroot_counter_ = -1; // Setting to -1 will reset the deltaroot during the next iteration
    return;
  }

  // THIS IS NOT THE CORRECT COVARIANCE - ITS THE COVARIANCE IN THE CAMERA FRAME!!!!
  // 5a. output the per-frame delta:
  vo_->send_delta_translation_msg(delta_body,
          delta_camera_cov, "VO_DELTA_BODY" );  
  // 5b. output the per-keyframe delta for the last period
  fovis::update_t update_msg = vo_->get_delta_translation_msg(delta_body_from_ref, delta_camera_cov, utime, deltaroot_utime_);
  lcm_pub_->publish("VO_DELTA_BODY_SINCE_REF", &update_msg);


  if (1==0){
    Isometry3dTime deltaroot_body_poseT = Isometry3dTime(deltaroot_utime_, deltaroot_body_pose_);
    pc_vis_->pose_to_lcm_from_list(3004, deltaroot_body_poseT);

    Isometry3dTime world_to_bodyT = Isometry3dTime(utime, world_to_body_);
    pc_vis_->pose_to_lcm_from_list(3009, world_to_bodyT);

    Isometry3dTime delta_body_from_refT = Isometry3dTime(utime, delta_body_from_ref);
    pc_vis_->pose_to_lcm_from_list(3006, delta_body_from_refT);
  }

}

/// Added for RGB-to-Gray:
int pixel_convert_8u_rgb_to_8u_gray (uint8_t *dest, int dstride, int width,
        int height, const uint8_t *src, int sstride)
{
  int i, j;
  for (i=0; i<height; i++) {
    uint8_t *drow = dest + i * dstride;
    const uint8_t *srow = src + i * sstride;
    for (j=0; j<width; j++) {
      drow[j] = 0.2125 * srow[j*3+0] +
        0.7154 * srow[j*3+1] +
        0.0721 * srow[j*3+2];
    }
  }
  return 0;
}

void StereoOdom::multisenseHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){
  frame_counter_++;
  if (frame_counter_% process_skip_  != 0){
    return;
  }

  if (!pose_initialized_){
    return;
  }

  utime_prev_ = utime_cur_;
  utime_cur_ = msg->utime;

  // Detect the image stream and process accordingly
  if ( (msg->image_types[0] ==  bot_core::images_t::LEFT) &&
       (msg->image_types[1] ==  bot_core::images_t::RIGHT) ) {

    multisenseLRHandler(rbuf, channel, msg);
    vo_->doOdometry(left_buf_,right_buf_, msg->utime);
  }else if( (msg->image_types[0] ==  bot_core::images_t::LEFT) &&
       (msg->image_types[1] ==  bot_core::images_t::DISPARITY_ZIPPED) ) {

    multisenseLDHandler(rbuf, channel, msg);
    vo_->doOdometry(left_buf_,disparity_buf_.data(), msg->utime );
  }else{
    std::cout << "StereoOdom::multisenseHandler | image pairings not understood\n";
    return;
  }
  updateMotion(msg->utime, utime_prev_);

  if(publish_feature_analysis_)
    featureAnalysis();

}


void StereoOdom::multisenseLDHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){

  int w = msg->images[0].width;
  int h = msg->images[0].height;

  if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY ){
    rgb_buf_ = (uint8_t*) msg->images[0].data.data();
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG ){
    jpeg_decompress_8u_rgb ( msg->images[0].data.data(), msg->images[0].size, rgb_buf_, w, h, w* 3);
    pixel_convert_8u_rgb_to_8u_gray(  left_buf_, w, w, h, rgb_buf_,  w*3);
  }else{
    std::cout << "StereoOdom image type not understood\n";
    exit(-1);
  }

  // TODO: support other modes (as in the renderer)
  if (msg->image_types[1] == bot_core::images_t::DISPARITY_ZIPPED) {
    unsigned long dlen = w*h*2;
    uncompress(decompress_disparity_buf_ , &dlen, msg->images[1].data.data(), msg->images[1].size);
  } else{
    std::cout << "StereoOdom depth type not understood\n";
    exit(-1);
  }

  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
  disparity_orig_temp.data = (uchar*) decompress_disparity_buf_;   // ... is a simple assignment possible?  
  cv::Mat_<float> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  disparity_buf_.resize(h * w);
  cv::Mat_<float> disparity(h, w, &(disparity_buf_[0]));
  disparity = disparity_orig / 16.0;


  if (filter_disparity_){
    // Filter the data to remove far away depth and the crash bar (from hyq)
    filterDisparity(msg, w, h);
  }
 
  return;
}

void StereoOdom::filterDisparity(const  bot_core::images_t* msg, int w, int h){
  // TODO: measure how long this takes, it can be implemented more efficiently

  for(int v=0; v<h; v++) { // t2b
    for(int u=0; u<w; u++ ) {  //l2r

      if (v > filter_image_rows_above_){
        disparity_buf_[w*v + u] = 0;
        //left_buf_[w*v + u] = 0;
      }else{
        float val = disparity_buf_[w*v + u];
        if (val < filter_disparity_below_threshold_){
          disparity_buf_[w*v + u] = 0;
          //left_buf_[w*v + u] = 100;
        } else if (val > filter_disparity_above_threshold_){
          disparity_buf_[w*v + u] = 0;
          //left_buf_[w*v + u] = 100;
        }
      }
    }
  }

  if (publish_filtered_image_)
    republishImage(msg);

}


void StereoOdom::republishImage(const  bot_core::images_t* msg){
  int width = msg->images[0].width;
  int height = msg->images[0].height;

  bot_core::image_t msgout_image;
  msgout_image.utime = msg->utime;
  msgout_image.width = width;
  msgout_image.height = height;
  msgout_image.row_stride = width;
  msgout_image.size = width*height;
  msgout_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY;
  msgout_image.data.resize( width*height );
  memcpy(msgout_image.data.data(), left_buf_, width*height );
  msgout_image.nmetadata =0;
  lcm_pub_->publish("CAMERA_LEFT_FILTERED", &msgout_image);

  bot_core::image_t msgout_depth;
  msgout_depth.utime = msg->utime;
  msgout_depth.width = width;
  msgout_depth.height = height;
  msgout_depth.row_stride = 2*width;
  msgout_depth.size = 2*width*height;
  msgout_depth.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY; // false, no info
  msgout_depth.data.resize( 2*width*height );
  memcpy(msgout_depth.data.data(), decompress_disparity_buf_, 2*width*height );
  msgout_depth.nmetadata =0;

  bot_core::images_t msgo;
  msgo.utime = msg->utime;
  msgo.n_images = 2;
  msgo.images.resize(2);
  msgo.image_types.resize(2);
  msgo.image_types[0] = bot_core::images_t::LEFT;
  msgo.image_types[1] = bot_core::images_t::DISPARITY;
  msgo.images[0] = msg->images[0];
  msgo.images[1] = msgout_depth;
  lcm_pub_->publish( "CAMERA_FILTERED" , &msgo);

  /*
  ofstream myfile;
  myfile.open ("example.txt");
  //myfile << "Writing this to a file.\n";
  for(int v=0; v<h; v++) { // t2b
    std::stringstream ss;
    for(int u=0; u<w; u++ ) {  //l2r
      ss << (float) disparity_buf_[w*v + u] << " ";
      // cout <<j2 << " " << v << " " << u << " | " <<  points(v,u)[0] << " " <<  points(v,u)[1] << " " <<  points(v,u)[1] << "\n";
      // std::cout <<  << " " << v << " " << u << "\n";
    }
    myfile << ss.str() << "\n";
  }
  myfile.close();
  std::cout << "writing\n";
  */
}


void StereoOdom::deltaResetHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::pose_t* msg){
  std::cout << "Got message on channel " << channel << ", will reset deltaroot\n";
  deltaroot_counter_ = -1; // Setting to -1 will reset the deltaroot during the next iteration
}


void StereoOdom::multisenseLRHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){

  int w = msg->images[0].width;
  int h = msg->images[0].height;

  if (msg->images[0].pixelformat != msg->images[1].pixelformat){
    std::cout << "Pixel formats not identical, not supported\n";
    exit(-1);
  }

  switch (msg->images[0].pixelformat) {
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY:
      memcpy(left_buf_,  msg->images[0].data.data() , msg->images[0].size);
      memcpy(right_buf_,  msg->images[1].data.data() , msg->images[1].size);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
      // image came in as raw RGB buffer.  convert to grayscale:
      pixel_convert_8u_rgb_to_8u_gray(  left_buf_ , w, w, h, msg->images[0].data.data(),  w*3);
      pixel_convert_8u_rgb_to_8u_gray(  right_buf_, w, w, h, msg->images[1].data.data(),  w*3);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // not sure why but the same setting seem to work for both jpeg compressed color (left) and grey (right):
      jpeg_decompress_8u_gray(msg->images[0].data.data(), msg->images[0].size,
                              left_buf_, w, h, w);
      jpeg_decompress_8u_gray(msg->images[1].data.data(), msg->images[1].size,
                              right_buf_, w, h, w);
      break;
    default:
      std::cout << "Unrecognized image format\n";
      exit(-1);
      break;
  }

  return;
}



static inline bot_core::pose_t getPoseAsBotPose(Eigen::Isometry3d pose, int64_t utime){
  bot_core::pose_t pose_msg;
  pose_msg.utime =   utime;
  pose_msg.pos[0] = pose.translation().x();
  pose_msg.pos[1] = pose.translation().y();
  pose_msg.pos[2] = pose.translation().z();  
  Eigen::Quaterniond r_x(pose.rotation());
  pose_msg.orientation[0] =  r_x.w();  
  pose_msg.orientation[1] =  r_x.x();  
  pose_msg.orientation[2] =  r_x.y();  
  pose_msg.orientation[3] =  r_x.z();  
  return pose_msg;
}



void StereoOdom::initialiseCameraPose(Eigen::Isometry3d world_to_body_init, int64_t utime){

  if (check_valid_camera_frame_){
    // Because body to CAMERA comes through FK, need to get an updated frame (BODY_TO_HEAD)
    int64_t timestamp;
    int status  = bot_frames_get_latest_timestamp(botframes_, 
                                        string( cl_cfg_.camera_config + "_LEFT" ).c_str() , "body", &timestamp);    
    if (timestamp==0){
      std::cout << "CAMERA_LEFT to body not updated, not initialising yet\n";
      std::cout << "This check if for articulated joints between head and body\n";
      return;
    }
  }

  Eigen::Isometry3d body_to_camera;
  get_trans_with_utime(botframes_, string( cl_cfg_.camera_config + "_LEFT" ).c_str() , "body", utime, body_to_camera);
  world_to_camera_ = world_to_body_init * body_to_camera;
  world_to_body_ = world_to_body_init;

  // Needed to set these frames properly at launch: 
  // TODO, this should be looked at again as I'm not sure this fully correct
  ref_camera_pose_ = world_to_camera_;
  ref_utime_ = utime;
  //
  deltaroot_body_pose_ = world_to_body_;
  deltaroot_utime_ = utime;

  std::cout << "Init state est using message\n";
  pose_initialized_ = TRUE;    

}


void StereoOdom::viconHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::rigid_transform_t* msg){

  Eigen::Isometry3d world_to_vicon_frame;
  world_to_vicon_frame.setIdentity();
  world_to_vicon_frame.translation()  << msg->trans[0], msg->trans[1] , msg->trans[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->quat[0], msg->quat[1], 
                                              msg->quat[2], msg->quat[3]);
  world_to_vicon_frame.rotate(quat); 

  // Apply the body to frontplate transform and find the camera:
  //Eigen::Isometry3d vicon_frame_to_body_vicon = botframes_cpp_->get_trans_with_utime(botframes_, "body_vicon" , "frontplate_vicon", msg->utime);
  Eigen::Isometry3d vicon_frame_to_body_vicon;
  get_trans_with_utime(botframes_, "body_vicon" , "vicon_frame", msg->utime, vicon_frame_to_body_vicon);
  Eigen::Isometry3d world_to_body_vicon = world_to_vicon_frame* vicon_frame_to_body_vicon;

  // Also publish back Vicon pose, if required
  bot_core::pose_t pose_msg = getPoseAsBotPose(world_to_body_vicon, msg->utime);
  lcm_pub_->publish("POSE_VICON", &pose_msg );

  if ( cl_cfg_.vicon_init && !pose_initialized_ ){ // Only Initialise once
    initialiseCameraPose(world_to_body_vicon, msg->utime);
  }
}

void StereoOdom::poseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::pose_t* msg){

  Eigen::Isometry3d world_to_body_alt;
  world_to_body_alt.setIdentity();
  world_to_body_alt.translation()  << msg->pos[0], msg->pos[1] , msg->pos[2];
  Eigen::Quaterniond quat = Eigen::Quaterniond(msg->orientation[0], msg->orientation[1], 
                                              msg->orientation[2], msg->orientation[3]);
  world_to_body_alt.rotate(quat); 

  if ( cl_cfg_.pose_init && !pose_initialized_ ){ // Only Initialise once
    initialiseCameraPose(world_to_body_alt, msg->utime);
  }
}




int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.camera_config = "MULTISENSE_CAMERA";
  cl_cfg.input_channel = "MULTISENSE_CAMERA";
  cl_cfg.output_signal = FALSE;
  cl_cfg.body_channel = "POSE_BODY_USING_CAMERA";
    cl_cfg.vicon_init = FALSE;
  cl_cfg.vicon_init_channel = "VICON_pelvis_val";
  cl_cfg.pose_init = FALSE;
  cl_cfg.pose_init_channel = "POSE_BODY_ALT";
  cl_cfg.fusion_mode = 0;
  cl_cfg.output_extension = "";
  cl_cfg.in_log_fname = "";
  std::string param_file = ""; // actual file
  cl_cfg.param_file = ""; // full path to file
  cl_cfg.draw_lcmgl = FALSE;

  ConciseArgs parser(argc, argv, "fovision-odometry");
  parser.add(cl_cfg.camera_config, "c", "camera_config", "Camera Config block to use: MULTISENSE_CAMERA, stereo, stereo_with_letterbox");
  parser.add(cl_cfg.output_signal, "p", "output_signal", "Output POSE_CAMERA_LEFT_ALT and body estimates");
  parser.add(cl_cfg.body_channel, "b", "body_channel", "body frame estimate (typically POSE_BODY)");
  parser.add(cl_cfg.vicon_init, "vi", "vicon_init", "Bootstrap internal estimate using a vicon rigid_transform_t msg");
  parser.add(cl_cfg.vicon_init_channel, "vc", "vicon_init_channel", "If initialising with a rigid_transform_t msg, use this channel");
  parser.add(cl_cfg.pose_init, "pi", "pose_init", "Bootstrap internal estimate using a pose_t message");
  parser.add(cl_cfg.pose_init_channel, "pc", "pose_init_channel", "If initialising with a pose message, use this channel");
  parser.add(cl_cfg.fusion_mode, "m", "fusion_mode", "0 none, 1 at init, 2 every second, 3 init from gt, then every second");
  parser.add(cl_cfg.input_channel, "i", "input_channel", "input_channel - MULTISENSE_CAMERA or CAMERA_BLACKENED");
  parser.add(cl_cfg.output_extension, "o", "output_extension", "Extension to pose channels (e.g. '_VO' ");
  parser.add(cl_cfg.in_log_fname, "L", "in_log_fname", "Process this log file");
  parser.add(param_file, "P", "param_file", "Pull params from this file instead of LCM");
  parser.add(cl_cfg.draw_lcmgl, "g", "lcmgl", "Draw LCMGL visualization of features");
  parser.parse();
  cout << cl_cfg.fusion_mode << " is fusion_mode\n";
  cout << cl_cfg.camera_config << " is camera_config\n";

  cl_cfg.param_file = std::string(getConfigPath()) +'/' + std::string(param_file);
  if (param_file.empty()) { // get param from lcm
    cl_cfg.param_file = "";
  }

  //
  bool running_from_log = !cl_cfg.in_log_fname.empty();
  boost::shared_ptr<lcm::LCM> lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_pub;
  if (running_from_log) {
    printf("running from log file: %s\n", cl_cfg.in_log_fname.c_str());
    //std::string lcmurl = "file://" + in_log_fname + "?speed=0";
    std::stringstream lcmurl;
    //lcmurl << "file://" << in_log_fname << "?speed=" << processing_rate << "&start_timestamp=" << begin_timestamp;
    lcmurl << "file://" << cl_cfg.in_log_fname ;
    lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcmurl.str()));
    if (!lcm_recv->good()) {
      fprintf(stderr, "Error couldn't load log file %s\n", lcmurl.str().c_str());
      exit(1);
    }
  }
  else {
    lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  }
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);

  StereoOdom fo= StereoOdom(lcm_recv, lcm_pub, cl_cfg);
  while(0 == lcm_recv->handle());
}
