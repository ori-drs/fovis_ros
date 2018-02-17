// A VO-based non-probablistic state estimator for the multisense
// - occasionally uses IMU to avoid orientation drift
// - when VO fails extrapolate using previous vision lin rate and imu rot rates

// For IMU orientation integration:
// Estimate is maintained in the body frame which is assumed to be
// Forward-Left-Up such at roll and pitch can be isolated from yaw.


#include <zlib.h>
#include <lcm/lcm-cpp.hpp>


#include <fovision_apps/fovision_fusion_core.hpp>


#include <pronto_vis/pronto_vis.hpp> // visualize pt clds
#include <image_io_utils/image_io_utils.hpp> // to simplify jpeg/zlib compression and decompression
#include <ConciseArgs>

#include <path_util/path_util.h>

#include <opencv/cv.h> // for disparity 

/////
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>


#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>


using namespace std;
using namespace cv; // for disparity ops

struct CommandLineConfig
{
  std::string input_channel;
  std::string imu_channel;
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
    StereoOdom(ros::NodeHandle node_in, boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_,
      const CommandLineConfig& cl_cfg_, const FusionCoreConfig& fcfg_);
    
    ~StereoOdom(){
    }

  private:
    const CommandLineConfig cl_cfg_;    
    boost::shared_ptr<lcm::LCM> lcm_recv_;
    boost::shared_ptr<lcm::LCM> lcm_pub_;
    FusionCore* vo_core_;

    // ROS:
    ros::NodeHandle node_;
    image_transport::ImageTransport it_;
    void head_stereo_cb(const sensor_msgs::ImageConstPtr& image_a_ros, const sensor_msgs::CameraInfoConstPtr& info_cam_a,
                      const sensor_msgs::ImageConstPtr& image_b_ros, const sensor_msgs::CameraInfoConstPtr& info_cam_b);
    void head_stereo_without_info_cb(const sensor_msgs::ImageConstPtr& image_a_ros, const sensor_msgs::ImageConstPtr& image_b_ros);
    image_transport::SubscriberFilter image_a_ros_sub_, image_b_ros_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_a_ros_sub_, info_b_ros_sub_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
        sensor_msgs::CameraInfo> sync_;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync_without_info_;
    tf::TransformBroadcaster br;


    ros::Subscriber imuSensorSub_;
    void imuSensorCallback(const sensor_msgs::ImuConstPtr& msg);


};    

StereoOdom::StereoOdom(ros::NodeHandle node_in,
       boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_,
       const CommandLineConfig& cl_cfg_, const FusionCoreConfig& fcfg_) : 
       node_(node_in), it_(node_in), sync_(10), sync_without_info_(10),
       lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_), cl_cfg_(cl_cfg_)
{
  vo_core_ = new FusionCore(lcm_recv_, lcm_pub_, fcfg_);


  std::string image_a_string, info_a_string, image_b_string, info_b_string;
  std::string head_stereo_root = "multisense";

  //image_a_string = head_stereo_root + "/left/image_rect_color";
  image_a_string = head_stereo_root + "/left/image_color";
  info_a_string = image_a_string + "/camera_info";

  bool output_right_image = false; // otherwise the disparity image
  if(output_right_image){
    //image_b_string = head_stereo_root + "/right/image_rect";
    image_b_string = head_stereo_root + "/right/image_mono";
    info_b_string = image_b_string + "/camera_info";
  }else{
    image_b_string = head_stereo_root + "/left/disparity";
    info_b_string = image_b_string + "/camera_info";
  }

  std::cout << image_a_string << " is the image_a topic subscription [for stereo]\n";
  std::cout << image_b_string << " is the image_b topic subscription [for stereo]\n";  
  image_a_ros_sub_.subscribe(it_, ros::names::resolve(image_a_string), 30);
  info_a_ros_sub_.subscribe(node_, ros::names::resolve(info_a_string), 30);
  image_b_ros_sub_.subscribe(it_, ros::names::resolve(image_b_string), 30);
  info_b_ros_sub_.subscribe(node_, ros::names::resolve(info_b_string), 30);
  //sync_.connectInput(image_a_ros_sub_, info_a_ros_sub_, image_b_ros_sub_, info_b_ros_sub_);
  //sync_.registerCallback(boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4));

  sync_without_info_.connectInput(image_a_ros_sub_, image_b_ros_sub_);
  sync_without_info_.registerCallback(boost::bind(&StereoOdom::head_stereo_without_info_cb , this, _1, _2));


  imuSensorSub_ = node_.subscribe(std::string("/imu/data"), 100,
                                    &StereoOdom::imuSensorCallback, this);  

  cout <<"StereoOdom Constructed\n";
}



int stereo_counter = 0;
void StereoOdom::head_stereo_cb(const sensor_msgs::ImageConstPtr& image_a_ros,
                         const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                         const sensor_msgs::ImageConstPtr& image_b_ros,
                         const sensor_msgs::CameraInfoConstPtr& info_b_ros)
{
/*  
  ROS_ERROR("AHDCAM [%d]", stereo_counter);
  int64_t current_utime = (int64_t)floor(image_a_ros->header.stamp.toNSec() / 1000);
  publishStereo(image_a_ros, info_a_ros, image_b_ros, info_b_ros, "MULTISENSE_CAMERA");

  if (stereo_counter % 30 == 0)
  {
    ROS_ERROR("HDCAM [%d]", stereo_counter);
  }
  stereo_counter++;
*/
}


void StereoOdom::head_stereo_without_info_cb(const sensor_msgs::ImageConstPtr& image_a_ros,
                         const sensor_msgs::ImageConstPtr& image_b_ros)
{
  if (!vo_core_->isPoseInitialized()){
    std::cout << "!pose_initialized. no imu input yet. will not compute VO\n";
    return;
  }
  int64_t utime_cur = (int64_t)floor(image_a_ros->header.stamp.toNSec() / 1000);
  vo_core_->setCurrentTime(utime_cur);

  double w = image_a_ros->width;
  double h = image_a_ros->height;
  double step_a = image_a_ros->step;
  double step_b = image_b_ros->step;

  if (image_a_ros->encoding != "bgr8")
    std::cout << "first image should be bgr8 (color). something is wrong!\n";

  if (image_b_ros->encoding != "mono16")
    std::cout << "second image should be mono16 (disparity). something is wrong!\n";


  if (image_b_ros->encoding == "mono16"){
    // this assumes this encoding is disparity. better to have different handlers I think

    void* left_data = const_cast<void*>(static_cast<const void*>(image_a_ros->data.data()));
    void* disp_data = const_cast<void*>(static_cast<const void*>(image_b_ros->data.data()));

    // TODO: use a bgr converter as it uses a different weighting of r and b
    memcpy(vo_core_->rgb_buf_, left_data, h*step_a);
    pixel_convert_8u_rgb_to_8u_gray(  vo_core_->left_buf_, w, w, h, vo_core_->rgb_buf_,  step_a);

    memcpy(vo_core_->decompress_disparity_buf_, disp_data, h*step_b); // 2 bytes

    // Convert Carnegie disparity format into floating point disparity. Store in local buffer
    Mat disparity_orig_temp = Mat::zeros(h,w,CV_16UC1); // h,w
    disparity_orig_temp.data = (uchar*) vo_core_->decompress_disparity_buf_;   // ... is a simple assignment possible?
    cv::Mat_<float> disparity_orig(h, w);
    disparity_orig = disparity_orig_temp;
    vo_core_->disparity_buf_.resize(h * w);
    cv::Mat_<float> disparity(h, w, &(vo_core_->disparity_buf_[0]));
    disparity = disparity_orig / 16.0;

    vo_core_->doOdometryLeftDisparity();

  }else{ 
    // mono grey scale. not fully supported yet

    void* left_data = const_cast<void*>(static_cast<const void*>(image_a_ros->data.data()));
    void* right_data = const_cast<void*>(static_cast<const void*>(image_b_ros->data.data()));
    memcpy(vo_core_->left_buf_, left_data, h*step_a);
    memcpy(vo_core_->right_buf_, right_data, h*step_b);
    vo_core_->doOdometryLeftRight();

  }


  vo_core_->doPostProcessing();


  tf::Transform transform;
  tf::poseEigenToTF( vo_core_->getBodyPose(), transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(utime_cur * 1E-6), "odom", "base_link"));


  tf::Transform transform2;
  Eigen::Isometry3d null = Eigen::Isometry3d::Identity();
  null.translation().x() = 0.465;
  null.translation().y() = 0.0;
  null.translation().z() = 0.558;

  Eigen::Quaterniond quat_pitch = euler_to_quat(0, 0.28278, 0); //about 16.2 degrees
  null.rotate( quat_pitch );

  tf::poseEigenToTF(null, transform2);
  br.sendTransform(tf::StampedTransform(transform2, ros::Time().fromSec(utime_cur * 1E-6), "base_link", "multisense/head"));




  if (stereo_counter % 30 == 0)
  {
    ROS_ERROR("VO frame received [%d]", stereo_counter);
  }
  stereo_counter++;
}




/*
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
*/




void StereoOdom::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg)
{
  Eigen::Quaterniond imu_orientation_from_imu(msg->orientation.w,msg->orientation.x,
                                              msg->orientation.y,msg->orientation.z);
  Eigen::Vector3d gyro = Eigen::Vector3d(0,0,0); // i have nothing for this yet
  int64_t utime_imu = (int64_t)floor(msg->header.stamp.toNSec() / 1000);

  Eigen::Quaterniond body_orientation_from_imu = vo_core_->imuOrientationToRobotOrientation(imu_orientation_from_imu);
  vo_core_->setBodyOrientationFromImu(body_orientation_from_imu, gyro, utime_imu);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "simple_fusion");

  CommandLineConfig cl_cfg;
  //cl_cfg.imu_channel = "IMU_MICROSTRAIN";
  //cl_cfg.input_channel = "MULTISENSE_CAMERA";

  FusionCoreConfig fcfg;
  fcfg.camera_config = "MULTISENSE_CAMERA";
  fcfg.output_signal = "POSE_BODY";
  fcfg.output_signal_at_10Hz = FALSE;
  fcfg.feature_analysis = FALSE; 
  fcfg.fusion_mode = 0;
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

  ConciseArgs parser(argc, argv, "simple-fusion");
  parser.add(fcfg.camera_config, "c", "camera_config", "Camera Config block to use: CAMERA, stereo, stereo_with_letterbox");
  parser.add(fcfg.output_signal, "p", "output_signal", "Output POSE_BODY and POSE_BODY_ALT signals");
  parser.add(fcfg.output_signal_at_10Hz, "s", "output_signal_at_10Hz", "Output POSE_BODY_10HZ on the camera CB");
  parser.add(fcfg.feature_analysis, "f", "feature_analysis", "Publish Feature Analysis Data");
  parser.add(fcfg.feature_analysis_publish_period, "fp", "feature_analysis_publish_period", "Publish features with this period");  
  parser.add(fcfg.fusion_mode, "m", "fusion_mode", "0 none, 1 at init, 2 rpy, 3 rp only, (both continuous)");
  //parser.add(cl_cfg.input_channel, "i", "input_channel", "input_channel");
  parser.add(fcfg.output_extension, "o", "output_extension", "Extension to pose channels (e.g. '_VO' ");
  parser.add(fcfg.correction_frequency, "y", "correction_frequency", "Correct the R/P every XX IMU measurements");
  parser.add(fcfg.verbose, "v", "verbose", "Verbose printf");
  //parser.add(cl_cfg.imu_channel, "imu", "imu_channel", "IMU channel to listen for");
  parser.add(fcfg.in_log_fname, "L", "in_log_fname", "Process this log file");
  parser.add(param_file, "P", "param_file", "Pull params from this file instead of LCM");
  parser.add(fcfg.draw_lcmgl, "g", "lcmgl", "Draw LCMGL visualization of features");
  parser.add(processing_rate, "r", "processing_rate", "Processing Rate from a log [0=ASAP, 1=realtime]");  
  parser.add(fcfg.write_feature_output, "fo", "write_feature_output", "Write feature poses, images to file");
  parser.add(fcfg.which_vo_options, "n", "which_vo_options", "Which set of VO options to use [1=slow,2=fast]");
  parser.parse();
  cout << fcfg.fusion_mode << " is fusion_mode\n";
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
  }
  else {
    lcm_recv = boost::shared_ptr<lcm::LCM>(new lcm::LCM);
  }
  lcm_pub = boost::shared_ptr<lcm::LCM>(new lcm::LCM);


  ros::NodeHandle nh;
  new StereoOdom(nh, lcm_recv, lcm_pub, cl_cfg, fcfg);
  ros::spin();
  //while(0 == lcm_recv->handle());
}
