// A VO-based non-probablistic state estimator for the multisense
// - occasionally uses IMU to avoid orientation drift
// - when VO fails extrapolate using previous vision lin rate and imu rot rates

// For IMU orientation integration:
// Estimate is maintained in the body frame which is assumed to be
// Forward-Left-Up such at roll and pitch can be isolated from yaw.


#include <zlib.h>
#include <opencv/cv.h> // for disparity 
#include <lcm/lcm-cpp.hpp>

#include <lcmtypes/bot_core.hpp>
#include <lcmtypes/pronto.hpp>
#include <lcmtypes/pronto/update_t.hpp>
#include <lcmtypes/ori/navigationframedata_t.hpp>

#include <jpeg_utils/jpeg-utils.hpp> // rgb to gray conversion
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <path_util/path_util.h>
#include <fovision_apps/fovision_fusion_core.hpp>
#include <ConciseArgs>

using namespace std;
using namespace cv; // for disparity ops


struct CommandLineConfig
{
  std::string input_channel;
  std::string imu_channel;
};


class StereoOdom{
  public:
    StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_, 
      const CommandLineConfig& cl_cfg_, const FusionCoreConfig& fcfg_);
    
    ~StereoOdom(){
    }

  private:
    const CommandLineConfig cl_cfg_;
    boost::shared_ptr<lcm::LCM> lcm_recv_;
    boost::shared_ptr<lcm::LCM> lcm_pub_;
    FusionCore* vo_core_;

    // VO from another source:
    void odometryHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  pronto::update_t* msg);
    // VO directly
    void multisenseHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLDHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);
    void multisenseLRHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::images_t* msg);

    // IMU
    void microstrainHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  bot_core::ins_t* msg);
    void oxtsHandler(const lcm::ReceiveBuffer* rbuf, const std::string& channel, const  ori::navigationframedata_t* msg);
};    

StereoOdom::StereoOdom(boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_,
       const CommandLineConfig& cl_cfg_, const FusionCoreConfig& fcfg_) : 
       lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_), cl_cfg_(cl_cfg_)
{
  vo_core_ = new FusionCore(lcm_recv_, lcm_pub_, fcfg_);

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

  cout <<"StereoOdom Constructed\n";
}


void StereoOdom::odometryHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  pronto::update_t* msg){

  if (!vo_core_->isPoseInitialized()){
    std::cout << "!pose_initialized. no imu input yet. will not handle incoming VO\n";
    return;
  }

  vo_core_->setCurrentTime(msg->timestamp);

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
  vo_core_->updatePosition(delta_camera);

  // correct roll and pitch using inertial (typically)
  // also publishes main pose message
  vo_core_->fuseInterial(vo_core_->getBodyOrientationFromImu(), msg->timestamp);
}


void StereoOdom::multisenseHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){

  if (!vo_core_->isPoseInitialized()){
    std::cout << "!pose_initialized. no imu input yet. will not compute VO\n";
    return;
  }
  vo_core_->setCurrentTime(msg->utime);

  // Detect the image stream and process accordingly
  if ( (msg->image_types[0] ==  bot_core::images_t::LEFT) &&
       (msg->image_types[1] ==  bot_core::images_t::RIGHT) ) {
    multisenseLRHandler(rbuf, channel, msg);
    vo_core_->doOdometryLeftRight();
  }else if( (msg->image_types[0] ==  bot_core::images_t::LEFT) &&
       (msg->image_types[1] ==  bot_core::images_t::DISPARITY_ZIPPED) ) {
    multisenseLDHandler(rbuf, channel, msg);
    vo_core_->doOdometryLeftDisparity();
  }else{
    std::cout << "StereoOdom::multisenseHandler | image pairings not understood\n";
    return;
  }

  vo_core_->doPostProcessing();
}


void StereoOdom::multisenseLDHandler(const lcm::ReceiveBuffer* rbuf,
     const std::string& channel, const  bot_core::images_t* msg){

  int w = msg->images[0].width;
  int h = msg->images[0].height;

  if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB ){
    vo_core_->rgb_buf_ = (uint8_t*) msg->images[0].data.data();
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_GRAY ){
    vo_core_->rgb_buf_ = (uint8_t*) msg->images[0].data.data();
  }else if (msg->images[0].pixelformat == BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG ){
    jpeg_decompress_8u_rgb ( msg->images[0].data.data(), msg->images[0].size, vo_core_->rgb_buf_, w, h, w* 3);
    pixel_convert_8u_rgb_to_8u_gray(  vo_core_->left_buf_, w, w, h, vo_core_->rgb_buf_,  w*3);
  }else{
    std::cout << "StereoOdom image type not understood\n";
    exit(-1);
  }

  // TODO: support other modes (as in the renderer)
  if (msg->image_types[1] == bot_core::images_t::DISPARITY_ZIPPED) {
    unsigned long dlen = w*h*2;
    uncompress(vo_core_->decompress_disparity_buf_ , &dlen, msg->images[1].data.data(), msg->images[1].size);
  } else{
    std::cout << "StereoOdom depth type not understood\n";
    exit(-1);
  }

  // Convert Carnegie disparity format into floating point disparity. Store in local buffer
  Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
  disparity_orig_temp.data = (uchar*) vo_core_->decompress_disparity_buf_;   // ... is a simple assignment possible?
  cv::Mat_<float> disparity_orig(h, w);
  disparity_orig = disparity_orig_temp;
  vo_core_->disparity_buf_.resize(h * w);
  cv::Mat_<float> disparity(h, w, &(vo_core_->disparity_buf_[0]));
  disparity = disparity_orig / 16.0;

  if ( vo_core_->isFilterDisparityEnabled() ){
    // Filter the data to remove far away depth and the crash bar (from hyq)
    vo_core_->filterDisparity(w, h);
  }

  return;
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
      memcpy(  vo_core_->left_buf_ ,  msg->images[0].data.data() , msg->images[0].size);
      memcpy(  vo_core_->right_buf_,  msg->images[1].data.data() , msg->images[1].size);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_RGB:
      // image came in as raw RGB buffer.  convert to grayscale:
      pixel_convert_8u_rgb_to_8u_gray(  vo_core_->left_buf_ , w, w, h, msg->images[0].data.data(),  w*3);
      pixel_convert_8u_rgb_to_8u_gray(  vo_core_->right_buf_, w, w, h, msg->images[1].data.data(),  w*3);
      break;
    case BOT_CORE_IMAGE_T_PIXEL_FORMAT_MJPEG:
      // not sure why but the same setting seem to work for both jpeg compressed color (left) and grey (right):
      jpeg_decompress_8u_gray(msg->images[0].data.data(), msg->images[0].size,
                              vo_core_->left_buf_ , w, h, w);
      jpeg_decompress_8u_gray(msg->images[1].data.data(), msg->images[1].size,
                              vo_core_->right_buf_, w, h, w);
      break;
    default:
      std::cout << "Unrecognized image format\n";
      exit(-1);
      break;
  }
 
  return;
}


void StereoOdom::microstrainHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  bot_core::ins_t* msg){

  Eigen::Quaterniond imu_orientation_from_imu(msg->quat[0],msg->quat[1],msg->quat[2],msg->quat[3]);
  Eigen::Vector3d gyro = Eigen::Vector3d(msg->gyro);

  Eigen::Quaterniond body_orientation_from_imu = vo_core_->imuOrientationToRobotOrientation(imu_orientation_from_imu);
  vo_core_->setBodyOrientationFromImu(body_orientation_from_imu, gyro, msg->utime);
}


void StereoOdom::oxtsHandler(const lcm::ReceiveBuffer* rbuf, 
     const std::string& channel, const  ori::navigationframedata_t* msg){

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

  Eigen::Quaterniond body_orientation_from_imu = Eigen::Quaterniond( iso.rotation() );

  Eigen::Vector3d gyro; // BUG: gyro rates not set. TODO: is there relevent data in the message?
  std::cout << "get gyro in oxtsHandler!\n";
  exit(-1);
  vo_core_->setBodyOrientationFromImu(body_orientation_from_imu, gyro, msg->utime);
}


int main(int argc, char **argv){
  CommandLineConfig cl_cfg;
  cl_cfg.input_channel = "MULTISENSE_CAMERA";
  cl_cfg.imu_channel = "IMU_MICROSTRAIN";

  FusionCoreConfig fcfg;
  fcfg.camera_config = "MULTISENSE_CAMERA";
  fcfg.output_signal = "POSE_BODY";
  fcfg.write_pose_to_file = FALSE;
  fcfg.publish_feature_analysis = FALSE; 
  fcfg.orientation_fusion_mode = 0;
  fcfg.verbose = false;
  fcfg.output_extension = "";
  fcfg.correction_frequency = 1;//; was typicall unused at 100;
  fcfg.feature_analysis_publish_period = 1; // 5
  std::string param_file = ""; // actual file
  fcfg.param_file = ""; // full path to file
  fcfg.draw_lcmgl = FALSE;  
  double processing_rate = 1; // real time
  fcfg.write_feature_output = FALSE;
  fcfg.which_vo_options = 2;

  std::cout << "Simple Fusion Modes (-m):\n";
  std::cout << "m=0 - initialise orientation using configuration file. do not use any IMU input (will drive in Roll and Pitch)\n";
  std::cout << "m=1 - initialise orientation using IMU. do not use IMU thereafter (useful for testing, will drift in Roll and Pitch)\n";
  std::cout << "m=2 - continously overwrite the orientation with that of the IMU (rarely used)\n";
  std::cout << "m=3 - continously overwrite the roll and pitch of the orientation with that of the IMU (commonly used for Husky, will not drive in Roll and Pitch)\n\n";

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(fcfg.camera_config, "c", "camera_config", "Camera Config block to use: CAMERA, stereo, stereo_with_letterbox");
  parser.add(fcfg.output_signal, "p", "output_signal", "Output POSE_BODY and POSE_BODY_ALT signals");
  parser.add(fcfg.write_pose_to_file, "s", "write_pose_to_file", "write_pose_to_file");
  parser.add(fcfg.publish_feature_analysis, "f", "publish_feature_analysis", "Publish Feature Analysis Data");
  parser.add(fcfg.feature_analysis_publish_period, "fp", "feature_analysis_publish_period", "Publish features with this period");  
  parser.add(fcfg.orientation_fusion_mode, "m", "orientation_fusion_mode", "0 none, 1 at init, 2 rpy, 3 rp only, (both continuous)");
  parser.add(cl_cfg.input_channel, "i", "input_channel", "input_channel");
  parser.add(fcfg.output_extension, "o", "output_extension", "Extension to pose channels (e.g. '_VO' ");
  parser.add(fcfg.correction_frequency, "y", "correction_frequency", "Correct the R/P every XX IMU measurements");
  parser.add(fcfg.verbose, "v", "verbose", "Verbose printf");
  parser.add(cl_cfg.imu_channel, "imu", "imu_channel", "IMU channel to listen for");
  parser.add(fcfg.in_log_fname, "L", "in_log_fname", "Process this log file");
  parser.add(param_file, "P", "param_file", "Pull params from this file instead of LCM");
  parser.add(fcfg.draw_lcmgl, "g", "lcmgl", "Draw LCMGL visualization of features");
  parser.add(processing_rate, "r", "processing_rate", "Processing Rate from a log [0=ASAP, 1=realtime]");  
  parser.add(fcfg.write_feature_output, "fo", "write_feature_output", "Write feature poses, images to file");
  parser.add(fcfg.which_vo_options, "n", "which_vo_options", "Which set of VO options to use [1=slow,2=fast]");
  parser.parse();
  cout << fcfg.orientation_fusion_mode << " is orientation_fusion_mode\n";
  cout << fcfg.camera_config << " is camera_config\n";
  
  fcfg.param_file = std::string(getConfigPath()) +'/' + std::string(param_file);
  if (param_file.empty()) { // get param from lcm
    fcfg.param_file = "";
  }

  //
  bool running_from_log = !fcfg.in_log_fname.empty();
  boost::shared_ptr<lcm::LCM> lcm_recv;
  boost::shared_ptr<lcm::LCM> lcm_pub;
  if (running_from_log) {
    printf("running from log file: %s\n", fcfg.in_log_fname.c_str());
    //std::string lcmurl = "file://" + in_log_fname + "?speed=0";
    std::stringstream lcmurl;
    lcmurl << "file://" << fcfg.in_log_fname << "?speed=" << processing_rate;
    lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM(lcmurl.str()));
    if (!lcm_recv->good()) {
      fprintf(stderr, "Error couldn't load log file %s\n", lcmurl.str().c_str());
      exit(1);
    }
  } else {
    lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  }
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);

  StereoOdom fo= StereoOdom(lcm_recv, lcm_pub, cl_cfg, fcfg);
  while(0 == lcm_recv->handle());
}
