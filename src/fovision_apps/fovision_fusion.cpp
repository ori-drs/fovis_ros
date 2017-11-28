// A VO-based non-probablistic state estimator for the multisense
// - occasionally uses IMU to avoid orientation drift
// - when VO fails extrapolate using previous vision lin rate and imu rot rates

// For IMU orientation integration:
// Estimate is maintained in the body frame which is assumed to be
// Forward-Left-Up such at roll and pitch can be isolated from yaw.


#include <zlib.h>
#include <lcm/lcm-cpp.hpp>
#include <bot_param/param_client.h>
#include <bot_frames/bot_frames.h>
//#include <bot_frames_cpp/bot_frames_cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/pronto.hpp>
#include <lcmtypes/pronto/update_t.hpp>
#include <lcmtypes/ori/navigationframedata_t.hpp>

#include "voconfig/voconfig.hpp"
#include "vofeatures/vofeatures.hpp"
#include "voestimator/voestimator.hpp"
#include "fovision/fovision.hpp"

#include <pronto_vis/pronto_vis.hpp> // visualize pt clds
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

#include <path_util/path_util.h>

#include <opencv/cv.h> // for disparity 

using namespace std;
using namespace cv; // for disparity ops

struct CommandLineConfig
{
  std::string camera_config; // which block from the cfg to read
  // 0 none, 1 at init, 2 rpy, 2 rp only
  int fusion_mode;
  bool feature_analysis;
  int feature_analysis_publish_period; // number of frames between publishing the point features 
  std::string output_extension;
  std::string output_signal;
  bool output_signal_at_10Hz;
  std::string input_channel;
  bool verbose;
  int correction_frequency;
  int atlas_version;
  std::string imu_channel;
  std::string in_log_fname;
  std::string param_file;
  bool draw_lcmgl;
  bool write_feature_output;
  int which_vo_options;
};

std::ofstream fovision_output_file_;


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
    void featureAnalysis();
    void updateMotion();

    // VO from another source:
    void odometryHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::update_t* msg);

    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);

    // IMU
    bool pose_initialized_; // initalized from VO
    int imu_counter_;
    void microstrainHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg);
    Eigen::Quaterniond imuOrientationToRobotOrientation(const bot_core::ins_t *msg);
    void fuseInterial(Eigen::Quaterniond local_to_body_orientation_from_imu, int64_t utime);
    Eigen::Isometry3d body_to_imu_, imu_to_camera_;
    
    void oxtsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  ori::navigationframedata_t* msg);

    // previous successful vo estimates as rates:
    Eigen::Vector3d vo_velocity_linear_;
    Eigen::Vector3d vo_velocity_angular_;
    Eigen::Vector3d camera_linear_velocity_from_imu_; // in camera frame
    Eigen::Vector3d camera_angular_velocity_from_imu_; // in camera frame
    Eigen::Vector3d camera_angular_velocity_from_imu_alpha_; // in camera frame

    // Most recent IMU-derived body orientation
    Eigen::Quaterniond local_to_body_orientation_from_imu_;

    // Filter the disparity image
    void filterDisparity(const  bot_core::images_t* msg, int w, int h);
    // Republish image to LCM. Used to examine the disparity filtering
    void republishImage(const  bot_core::images_t* msg);

    // Image prefiltering
    bool filter_disparity_;
    double filter_disparity_below_threshold_;
    double filter_disparity_above_threshold_;
    int filter_image_rows_above_;
    bool publish_filtered_image_;

};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, const CommandLineConfig& cl_cfg_) : 
       lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_), cl_cfg_(cl_cfg_), utime_cur_(0), utime_prev_(0), 
       ref_utime_(0), changed_ref_frames_(false)
{
  // Set up frames and config:
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
  botframes_= bot_frames_get_global(lcm_recv_->getUnderlyingLCM(), botparam_);
  //botframes_cpp_ = new bot::frames(botframes_);

  config_ = new voconfig::KmclConfiguration(botparam_, cl_cfg_.camera_config);
  boost::shared_ptr<fovis::StereoCalibration> stereo_calibration_;
  stereo_calibration_ = boost::shared_ptr<fovis::StereoCalibration>(config_->load_stereo_calibration());

  // Disparity filtering
  filter_disparity_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.filter.enabled");
  std::cout << "Disparity Filter is " << (filter_disparity_ ? "ENABLED" : "DISABLED")  << "\n";
  filter_disparity_below_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_disparity_below_threshold");
  filter_disparity_above_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_disparity_above_threshold");
  filter_image_rows_above_ = bot_param_get_int_or_fail(botparam_, "visual_odometry.filter.filter_image_rows_above");
  publish_filtered_image_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.filter.publish_filtered_image");

  // Allocate various buffers:
  image_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  left_buf_ = (uint8_t*) malloc(3*image_size_);
  right_buf_ = (uint8_t*) malloc(3*image_size_);
  left_buf_ref_ = (uint8_t*) malloc(3*image_size_); // used of feature output 
  rgb_buf_ = (uint8_t*) malloc(10*image_size_ * sizeof(uint8_t)); 
  decompress_disparity_buf_ = (uint8_t*) malloc( 4*image_size_*sizeof(uint8_t));  // arbitary size chosen..
  imgutils_ = new image_io_utils( lcm_pub_, stereo_calibration_->getWidth(), 2*stereo_calibration_->getHeight()); // extra space for stereo tasks

  vo_ = new FoVision(lcm_pub_ , stereo_calibration_, cl_cfg_.draw_lcmgl, cl_cfg_.which_vo_options);
  features_ = new VoFeatures(lcm_pub_, stereo_calibration_->getWidth(), stereo_calibration_->getHeight() );
  estimator_ = new VoEstimator(lcm_pub_ , botframes_, cl_cfg_.output_extension, cl_cfg_.camera_config );

  Eigen::Isometry3d init_pose;
  init_pose = Eigen::Isometry3d::Identity();
  init_pose.translation() = Eigen::Vector3d(0,0,0); // set nominal head height
  Eigen::Quaterniond quat = euler_to_quat(0,0.29,0); // nominal pitch and roll
  init_pose.rotate(quat);
  estimator_->setBodyPose(init_pose);

  // IMU:
  pose_initialized_=false;
  imu_counter_=0;

  if (cl_cfg_.input_channel == "STEREO_ODOMETRY"){
    std::cout << "subscribing to "<< cl_cfg_.input_channel <<" for relative VO measurements\n";
    lcm_recv_->subscribe( cl_cfg_.input_channel, &StereoOdom::odometryHandler,this);
  }else{
    std::cout << "subscribing to " << cl_cfg_.input_channel << " for images and will do VO\n";
    lcm_recv_->subscribe( cl_cfg_.input_channel, &StereoOdom::multisenseHandler,this);
  }

  if (cl_cfg_.imu_channel == "OXTS"){
    lcm_recv_->subscribe( cl_cfg_.imu_channel, &StereoOdom::oxtsHandler,this);
  }else{
    lcm_recv_->subscribe( cl_cfg_.imu_channel, &StereoOdom::microstrainHandler,this);
  }


  // This assumes the imu to body frame is fixed, need to update if the neck is actuated
  get_trans_with_utime( botframes_ ,  "body", "imu", 0, body_to_imu_);
  get_trans_with_utime( botframes_ ,  "imu",  string( cl_cfg_.camera_config + "_LEFT" ).c_str(), 0, imu_to_camera_);

  if( cl_cfg_.output_signal_at_10Hz ){
    std::cout << "Opened fovision_pose_body.txt\n";
    fovision_output_file_.open("fovision_pose_body.txt");
    fovision_output_file_ << "# utime x y z qw qx qy qz roll pitch yaw\n";
  }

  cout <<"StereoOdom Constructed\n";
}


void StereoOdom::odometryHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  pronto::update_t* msg){
  //  std::cout << "got vo"<< msg->timestamp<<"\n";

  if (!pose_initialized_){
    fuseInterial(local_to_body_orientation_from_imu_, msg->timestamp);
    return;
  }

  utime_prev_ = msg->prev_timestamp;
  utime_cur_ = msg->timestamp;

  // Update current estimate using vo for odometry
  Eigen::Isometry3d delta_camera;
  Eigen::Quaterniond quat;
  quat.w() = msg->rotation[0];  quat.x() = msg->rotation[1];
  quat.y() = msg->rotation[2];  quat.z() = msg->rotation[3];
  delta_camera.setIdentity();
  delta_camera.translation().x() = msg->translation[0];
  delta_camera.translation().y() = msg->translation[1];
  delta_camera.translation().z() = msg->translation[2];
  delta_camera.rotate(quat);
  estimator_->updatePosition(utime_cur_, utime_prev_, delta_camera);

  // correct roll and pitch using inertial (typically)
  // also publishes main pose message
  fuseInterial(local_to_body_orientation_from_imu_, msg->timestamp);
}


int counter =0;
void StereoOdom::featureAnalysis(){

  /// Incremental Feature Output:
  if (counter% cl_cfg_.feature_analysis_publish_period == 0 ){
    features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , utime_cur_);
    features_->setCurrentImage(left_buf_);
    features_->setCurrentCameraPose( estimator_->getCameraPose() );
    if (vo_->getNumMatches()>0)
      features_->doFeatureProcessing(true, cl_cfg_.write_feature_output ); // use current features
    else
      std::cout << "no matches, will not publish or output\n"; // this happens for first frame
  }
  
  /// Reference Feature Output: ///////////////////////////////////////////////
  // Check we changed reference frame last iteration, if so output the set of matching inliers:
  if (changed_ref_frames_) {
    if (ref_utime_ > 0){ // skip the first null image
      if(vo_->getNumMatches() > 200){ // if less than 50 features - dont bother writing
      // was:      if(featuresA.size() > 50){ // if less than 50 features - dont bother writing
        cout << "ref frame from " << utime_prev_ << " at " << utime_cur_ <<  " with " <<vo_->getNumMatches()<<" matches\n";
        features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , ref_utime_);
        features_->setReferenceImage(left_buf_ref_);
        features_->setReferenceCameraPose( ref_camera_pose_ );
        //features_->doFeatureProcessing(false); // use reference features
      }
    }
    changed_ref_frames_=false;
  }

  if (vo_->getChangeReferenceFrames()){ // If we change reference frame, note the change for the next iteration.
    ref_utime_ = utime_cur_;
    ref_camera_pose_ = estimator_->getCameraPose(); // publish this pose when the 
    // TODO: only copy gray data if its grey
    std::copy( left_buf_ , left_buf_ + 3*image_size_  , left_buf_ref_); // Keep the image buffer to write with the features:
    changed_ref_frames_=true;
  }
  counter++;
}

void StereoOdom::updateMotion(){
  // 1. Estimate the motion using VO
  Eigen::Isometry3d delta_camera;
  Eigen::MatrixXd delta_cov;
  fovis::MotionEstimateStatusCode delta_status;
  vo_->getMotion(delta_camera, delta_cov, delta_status );
  vo_->fovis_stats();

  // 2. If successful cache the rates
  //    otherwise extrapolate the previous rate
  if (delta_status == fovis::SUCCESS){
    // Get the 1st order rates:
    double dt = (double) ((utime_cur_ - utime_prev_)*1E-6);
    vo_velocity_linear_ = Eigen::Vector3d( delta_camera.translation().x() / dt ,
                                           delta_camera.translation().y() / dt ,
                                           delta_camera.translation().z() / dt);
    double rpy[3];
    quat_to_euler(  Eigen::Quaterniond(delta_camera.rotation()) , rpy[0], rpy[1], rpy[2]);
    vo_velocity_angular_ = Eigen::Vector3d( rpy[0]/dt , rpy[1]/dt , rpy[2]/dt);

    ////std::cout << vo_velocity_linear_.transpose() << " vo linear\n";
    //Eigen::Vector3d head_velocity_linear = estimator_->getBodyLinearRate();
    //std::cout << head_velocity_linear.transpose() << " vo linear head\n";

    //std::cout << vo_velocity_angular_.transpose() << " vo angular\n";
    //Eigen::Vector3d head_velocity_angular = estimator_->getBodyRotationRate();
    //std::cout << head_velocity_angular.transpose() << " vo angular head\n";

  }else{
    double dt = (double) ((utime_cur_ - utime_prev_)*1E-6);
    std::cout << "failed VO\n";
    if (fabs(dt) > 0.2){
      delta_camera.setIdentity();
      std::cout << "================ Unexpected jump: " << dt << " sec. Not extrapolating ==========\n";
    }else{

      // This orientation is not mathematically correct:
      std::cout << dt << " sec | "
                << vo_velocity_linear_.transpose()   << " m/s | "
                << camera_angular_velocity_from_imu_.transpose() << " r/s to be extrapolated\n";

      // Use IMU rot_rates to extrapolate rotation: This is mathematically incorrect:
      Eigen::Quaterniond extrapolated_quat = euler_to_quat( camera_angular_velocity_from_imu_[0]*dt, camera_angular_velocity_from_imu_[1]*dt, camera_angular_velocity_from_imu_[2]*dt);

      delta_camera.setIdentity();
      delta_camera.translation().x() = vo_velocity_linear_[0] * dt;
      delta_camera.translation().y() = vo_velocity_linear_[1] * dt;
      delta_camera.translation().z() = vo_velocity_linear_[2] * dt;
      delta_camera.rotate(extrapolated_quat);
    }

  }
  if (cl_cfg_.verbose){
    std::stringstream ss2;
    print_Isometry3d(delta_camera, ss2);
    std::cout << "camera: " << ss2.str() << " code" << (int) delta_status << "\n";
  }

  // 3. Update the motion estimation:
  estimator_->updatePosition(utime_cur_, utime_prev_, delta_camera);
}

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

  if (!pose_initialized_){
    fuseInterial(local_to_body_orientation_from_imu_, msg->utime);
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
  updateMotion();

  if(cl_cfg_.feature_analysis)
    featureAnalysis();


  if (cl_cfg_.output_signal_at_10Hz){
    // Publish the current estimate
    Eigen::Isometry3d local_to_body = estimator_->getBodyPose();
    estimator_->publishUpdate(utime_cur_, local_to_body, "POSE_BODY_10HZ", false);
    //lcm_pub_->publish("CAMERA_REPUBLISH", msg);

    double current_rpy[3];
    Eigen::Quaterniond current_quat = Eigen::Quaterniond( local_to_body.rotation() );
    quat_to_euler( current_quat, current_rpy[0], current_rpy[1], current_rpy[2]);
    //# utime, x, y, z, qw, qx, qy, qz, roll, pitch, yaw
    fovision_output_file_ << utime_cur_  << " "
                    << local_to_body.translation().x() << " " << local_to_body.translation().y() << " " << local_to_body.translation().z() << " "
                    << current_quat.w() << " " << current_quat.x() << " " << current_quat.y() << " " << current_quat.z() << " "
                    << current_rpy[0] << " " << current_rpy[1] << " " << current_rpy[2] << "\n";
    fovision_output_file_.flush();
  }

  fuseInterial(local_to_body_orientation_from_imu_, msg->utime);


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
        left_buf_[w*v + u] = 0;
      }else{
        float val = disparity_buf_[w*v + u];
        if (val < filter_disparity_below_threshold_ || val > filter_disparity_above_threshold_){
          disparity_buf_[w*v + u] = 0;
          left_buf_[w*v + u] = 100;
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
  lcm_pub_->publish("MULTISENSE_CAMERA_LEFT_FILTERED", &msgout_image);

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
  msgo.images[0] = msgout_image;
  msgo.images[1] = msgout_depth;
  lcm_pub_->publish( "MULTISENSE_CAMERA_FILTERED" , &msgo);

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


// Transform the Microstrain IMU orientation into the body frame:
Eigen::Quaterniond StereoOdom::imuOrientationToRobotOrientation(const bot_core::ins_t *msg){
  Eigen::Quaterniond m(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
  Eigen::Isometry3d motion_estimate;
  motion_estimate.setIdentity();
  motion_estimate.translation() << 0,0,0;
  motion_estimate.rotate(m);

  // rotate IMU orientation so that look vector is +X, and up is +Z
  // TODO: do this with rotation matrices for efficiency:
  Eigen::Isometry3d body_pose_from_imu = motion_estimate * body_to_imu_;
  Eigen::Quaterniond body_orientation_from_imu(body_pose_from_imu.rotation());
  // For debug:
  //estimator_->publishPose(msg->utime, "POSE_BODY" , motion_estimate_out, Eigen::Vector3d::Identity() , Eigen::Vector3d::Identity());
  //estimator_->publishPose(msg->utime, "POSE_BODY_ALT" , motion_estimate, Eigen::Vector3d::Identity() , Eigen::Vector3d::Identity());

  return body_orientation_from_imu;
}

void StereoOdom::fuseInterial(Eigen::Quaterniond local_to_body_orientation_from_imu, int64_t utime){

  if (cl_cfg_.fusion_mode==0) // Got IMU measurement - not incorporating them.
    return;
  
  if (!pose_initialized_){
    std::vector<int> init_modes = {1,2,3};
    if(std::find(init_modes.begin(), init_modes.end(), cl_cfg_.fusion_mode) != init_modes.end()) {
    //if((cl_cfg_.fusion_mode ==1) ||(cl_cfg_.fusion_mode ==2)  ){
      Eigen::Isometry3d init_pose;
      init_pose.setIdentity();
      init_pose.translation() << 0,0,0;

      // Take the RPY from the IMU and only use the roll+pitch with zero yaw to init the estimator
      double rpy_imu[3];
      quat_to_euler(  local_to_body_orientation_from_imu , rpy_imu[0], rpy_imu[1], rpy_imu[2]);
      init_pose.rotate( euler_to_quat( rpy_imu[0], rpy_imu[1], 0) );
      estimator_->setBodyPose(init_pose);
      pose_initialized_ = true;
      cout << "got first IMU measurement\n";
      return;
    }
  }

  if((cl_cfg_.fusion_mode ==2) ||(cl_cfg_.fusion_mode ==3) ){
    if (imu_counter_== cl_cfg_.correction_frequency){
      // Every X frames: replace the pitch and roll with that from the IMU
      // convert the camera pose to head frame
      // extract xyz and yaw from head frame
      // extract pitch and roll from imu (in head frame)
      // combine, convert to camera frame... set as pose

      // 1. Get the currently estimated head pose and its rpy
      Eigen::Isometry3d local_to_body = estimator_->getBodyPose();
      std::stringstream ss2;
      print_Isometry3d(local_to_body, ss2);
      double rpy[3];
      quat_to_euler(  Eigen::Quaterniond(local_to_body.rotation()) , rpy[0], rpy[1], rpy[2]);
      if (cl_cfg_.verbose){
        std::cout << "local_to_body: " << ss2.str() << " | "<< 
          rpy[0]*180/M_PI << " " << rpy[1]*180/M_PI << " " << rpy[2]*180/M_PI << "\n";        
      }
        
      // 2. Get the IMU orientated RPY:
      double rpy_imu[3];
      quat_to_euler( local_to_body_orientation_from_imu , 
                      rpy_imu[0], rpy_imu[1], rpy_imu[2]);
      if (cl_cfg_.verbose){
        std::cout <<  rpy_imu[0]*180/M_PI << " " << rpy_imu[1]*180/M_PI << " " << rpy_imu[2]*180/M_PI << " rpy_imu\n";        
        cout << "IMU correction | roll pitch | was: "
            << rpy[0]*180/M_PI << " " << rpy[1]*180/M_PI << " | now: "
            << rpy_imu[0]*180/M_PI << " " << rpy_imu[1]*180/M_PI << "\n";
      }
      
      // 3. Merge the two orientation estimates:
      Eigen::Quaterniond revised_local_to_body_quat;
      if (cl_cfg_.fusion_mode==2){ // rpy:
        revised_local_to_body_quat = local_to_body_orientation_from_imu;
      }else{  // pitch and roll from IMU, yaw from VO:
        revised_local_to_body_quat = euler_to_quat( rpy_imu[0], rpy_imu[1], rpy[2]);
      }
      ///////////////////////////////////////
      Eigen::Isometry3d revised_local_to_body;
      revised_local_to_body.setIdentity();
      revised_local_to_body.translation() = local_to_body.translation();
      revised_local_to_body.rotate(revised_local_to_body_quat);
      
      // 4. Set the Head pose using the merged orientation:
      if (cl_cfg_.verbose){
        std::stringstream ss4;
        print_Isometry3d(revised_local_to_body, ss4);
        quat_to_euler(  Eigen::Quaterniond(revised_local_to_body.rotation()) , rpy[0], rpy[1], rpy[2]);
        std::cout << "local_revhead: " << ss4.str() << " | "<< 
          rpy[0]*180/M_PI << " " << rpy[1]*180/M_PI << " " << rpy[2]*180/M_PI << "\n";        
      }
      estimator_->setBodyPose(revised_local_to_body);
    }
    if (imu_counter_ > cl_cfg_.correction_frequency) { imu_counter_ =0; }
    imu_counter_++;

    // Publish the Position of the floating head:
    estimator_->publishUpdate(utime_cur_, estimator_->getBodyPose(), cl_cfg_.output_signal, false);

  }
}


int temp_counter = 0;
void StereoOdom::microstrainHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  bot_core::ins_t* msg){
  temp_counter++;
  if (temp_counter > 5){
    //std::cout << msg->gyro[0] << ", " << msg->gyro[1] << ", " << msg->gyro[2] << " rotation rate, gyro frame\n";
    temp_counter=0;
  }

  local_to_body_orientation_from_imu_ = imuOrientationToRobotOrientation(msg);

  // Transform rotation Rates into body frame:
  double camera_ang_vel_from_imu_[3];
  Eigen::Quaterniond imu_to_camera_quat = Eigen::Quaterniond( imu_to_camera_.rotation() );
  double imu_to_camera_quat_array[4];
  imu_to_camera_quat_array[0] =imu_to_camera_quat.w(); imu_to_camera_quat_array[1] =imu_to_camera_quat.x();
  imu_to_camera_quat_array[2] =imu_to_camera_quat.y(); imu_to_camera_quat_array[3] =imu_to_camera_quat.z();
  bot_quat_rotate_to( imu_to_camera_quat_array, msg->gyro, camera_ang_vel_from_imu_);
  camera_angular_velocity_from_imu_ = Eigen::Vector3d(camera_ang_vel_from_imu_[0], camera_ang_vel_from_imu_[1], camera_ang_vel_from_imu_[2]);
  // Didn't find this necessary - for smooth motion
  //camera_angular_velocity_from_imu_alpha_ = 0.8*camera_angular_velocity_from_imu_alpha_ + 0.2*camera_angular_velocity_from_imu_;

  camera_linear_velocity_from_imu_  = Eigen::Vector3d(0,0,0);

  // experimentally correct for sensor timing offset:
  int64_t temp_utime = msg->utime;// + 120000;
  estimator_->publishPose(temp_utime, "POSE_IMU_RATES", Eigen::Isometry3d::Identity(), camera_linear_velocity_from_imu_, camera_angular_velocity_from_imu_);
  // estimator_->publishPose(temp_utime, "POSE_IMU_RATES", Eigen::Isometry3d::Identity(), camera_linear_velocity_from_imu_, camera_angular_velocity_from_imu_alpha_);
}





//int temp_counter_oxts = 0;
void StereoOdom::oxtsHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  ori::navigationframedata_t* msg){
  //temp_counter_oxts++;
  //if (temp_counter_oxts > 100){
  //  temp_counter_oxts=0;
  //  //std::cout << "got oxts\n";
  //}

  // convert to roll, pitch and yaw using assumed conventions:
  Eigen::Quaterniond quat = euler_to_quat(msg->roll,-msg->pitch,M_PI/2-msg->yaw);

  Eigen::Isometry3d iso;
  iso.setIdentity();
  // optionally remove the x,y,z offset - for ease of use:
  iso.translation().x() = msg->utm_easting - 601907;
  iso.translation().y() = msg->utm_northing - 5745580;
  iso.translation().z() = -msg->utm_down -138;
  iso.rotate(quat);

  // since body frame is actually camera for wildcat, then rotate by the imu-to-camera rotation
  // this is pretty rough:
  Eigen::Quaterniond quat_pitch = euler_to_quat(0, 0.2075, 0); //about 12 degrees
  iso.rotate( quat_pitch );

  local_to_body_orientation_from_imu_ = Eigen::Quaterniond( iso.rotation() );

  /*
  double rpy[3];
  quat_to_euler(  local_to_body_orientation_from_imu_ , rpy[0], rpy[1], rpy[2]);
  std::cout << rpy[0]*180.0/M_PI << ", "
            << rpy[1]*180.0/M_PI << ", "
            << rpy[2]*180.0/M_PI << "\n";
  */

// Use the GPS as the source of pose body
//  estimator_->publishPose(msg->utime, "POSE_BODY", iso, Eigen::Vector3d::Identity(), Eigen::Vector3d::Identity());
}



int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.camera_config = "MULTISENSE_CAMERA";
  cl_cfg.input_channel = "MULTISENSE_CAMERA";
  cl_cfg.output_signal = "POSE_BODY";
  cl_cfg.output_signal_at_10Hz = FALSE;
  cl_cfg.feature_analysis = FALSE; 
  cl_cfg.fusion_mode = 0;
  cl_cfg.verbose = false;
  cl_cfg.output_extension = "";
  cl_cfg.correction_frequency = 1;//; was typicall unused at 100;
  cl_cfg.atlas_version = 5;
  cl_cfg.feature_analysis_publish_period = 1; // 5
  cl_cfg.imu_channel = "IMU_MICROSTRAIN";
  std::string param_file = ""; // actual file
  cl_cfg.param_file = ""; // full path to file
  cl_cfg.draw_lcmgl = FALSE;  
  double processing_rate = 1; // real time
  cl_cfg.write_feature_output = FALSE;
  cl_cfg.which_vo_options = 2;

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(cl_cfg.camera_config, "c", "camera_config", "Camera Config block to use: CAMERA, stereo, stereo_with_letterbox");
  parser.add(cl_cfg.output_signal, "p", "output_signal", "Output POSE_BODY and POSE_BODY_ALT signals");
  parser.add(cl_cfg.output_signal_at_10Hz, "s", "output_signal_at_10Hz", "Output POSE_BODY_10HZ on the camera CB");
  parser.add(cl_cfg.feature_analysis, "f", "feature_analysis", "Publish Feature Analysis Data");
  parser.add(cl_cfg.feature_analysis_publish_period, "fp", "feature_analysis_publish_period", "Publish features with this period");  
  parser.add(cl_cfg.fusion_mode, "m", "fusion_mode", "0 none, 1 at init, 2 rpy, 3 rp only, (both continuous)");
  parser.add(cl_cfg.input_channel, "i", "input_channel", "input_channel");
  parser.add(cl_cfg.output_extension, "o", "output_extension", "Extension to pose channels (e.g. '_VO' ");
  parser.add(cl_cfg.correction_frequency, "y", "correction_frequency", "Correct the R/P every XX IMU measurements");
  parser.add(cl_cfg.verbose, "v", "verbose", "Verbose printf");
  parser.add(cl_cfg.imu_channel, "imu", "imu_channel", "IMU channel to listen for");
  parser.add(cl_cfg.atlas_version, "a", "atlas_version", "Atlas version to use");
  parser.add(cl_cfg.in_log_fname, "L", "in_log_fname", "Process this log file");
  parser.add(param_file, "P", "param_file", "Pull params from this file instead of LCM");
  parser.add(cl_cfg.draw_lcmgl, "g", "lcmgl", "Draw LCMGL visualization of features");
  parser.add(processing_rate, "r", "processing_rate", "Processing Rate from a log [0=ASAP, 1=realtime]");  
  parser.add(cl_cfg.write_feature_output, "fo", "write_feature_output", "Write feature poses, images to file");
  parser.add(cl_cfg.which_vo_options, "n", "which_vo_options", "Which set of VO options to use [1=slow,2=fast]");
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
    lcmurl << "file://" << cl_cfg.in_log_fname << "?speed=" << processing_rate;
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
