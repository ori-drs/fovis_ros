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

#include <path_util/path_util.h>

#include <opencv/cv.h> // for disparity 

/////
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>


#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>


using namespace std;
using namespace cv; // for disparity ops

class StereoOdom{
  public:
    StereoOdom(ros::NodeHandle node_in, boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_,
      FusionCoreConfig fcfg_);
    
    ~StereoOdom(){
    }

  private:
    const FusionCoreConfig fcfg_;
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

    ros::Publisher body_pose_pub_;

    int64_t utime_imu_;

    bool output_using_imu_time_; 
};    

StereoOdom::StereoOdom(ros::NodeHandle node_in,
       boost::shared_ptr<lcm::LCM> &lcm_recv_, boost::shared_ptr<lcm::LCM> &lcm_pub_,
       FusionCoreConfig fcfg_) : 
       node_(node_in), it_(node_in), sync_(10), sync_without_info_(10),
       lcm_recv_(lcm_recv_), lcm_pub_(lcm_pub_), fcfg_(fcfg_)
{

  vo_core_ = new FusionCore(lcm_recv_, lcm_pub_, fcfg_);

  // Parameters:
  output_using_imu_time_=true;
  if (!node_.getParam("output_using_imu_time", output_using_imu_time_)) {
    ROS_ERROR("Note: Could not read parameter `output_using_imu_time`. Enabled by default");
  }
  ROS_INFO("output_using_imu_time = %d", output_using_imu_time_);


  std::string image_a_string, image_b_string;
  std::string image_a_transport, image_b_transport;
  std::string info_a_string, info_b_string;


  if (!node_.getParam("image_a_topic", image_a_string)) {
    ROS_ERROR("Could not read `image_a_topic`.");
    exit(-1);
  }
  ROS_INFO( "%s is the image_a topic subscription [for stereo]", image_a_string.c_str());
  info_a_string = image_a_string + "/camera_info";
  if (!node_.getParam("image_a_transport", image_a_transport)) {
    ROS_ERROR("Could not read `image_a_transport`.");
    exit(-1);
  }
  ROS_INFO( "%s is the image_a_transport transport", image_a_transport.c_str());

  if (!node_.getParam("image_b_topic", image_b_string)) {
    ROS_ERROR("Could not read `image_b_string`.");
    exit(-1);
  }
  ROS_INFO( "%s is the image_b topic subscription [for stereo]", image_b_string.c_str());
  info_b_string = image_b_string + "/camera_info";
  if (!node_.getParam("image_b_transport", image_b_transport)) {
    ROS_ERROR("Could not read `image_b_transport`.");
    exit(-1);
  }
  ROS_INFO( "%s is the image_b_transport transport", image_b_transport.c_str());


  image_a_ros_sub_.subscribe(it_, ros::names::resolve(image_a_string), 30, image_transport::TransportHints( image_a_transport ));
  image_b_ros_sub_.subscribe(it_, ros::names::resolve(image_b_string), 30, image_transport::TransportHints( image_b_transport ));


  //info_a_ros_sub_.subscribe(node_, ros::names::resolve(info_a_string), 30);
  //info_b_ros_sub_.subscribe(node_, ros::names::resolve(info_b_string), 30);
  //sync_.connectInput(image_a_ros_sub_, info_a_ros_sub_, image_b_ros_sub_, info_b_ros_sub_);
  //sync_.registerCallback(boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4));
  sync_without_info_.connectInput(image_a_ros_sub_, image_b_ros_sub_);
  sync_without_info_.registerCallback(boost::bind(&StereoOdom::head_stereo_without_info_cb , this, _1, _2));


  std::string output_body_pose_topic;
  node_.getParam("output_body_pose_topic", output_body_pose_topic); 
  body_pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_body_pose_topic, 10);


  std::string imu_topic;
  node_.getParam("imu_topic", imu_topic); 
  imuSensorSub_ = node_.subscribe(std::string(imu_topic), 100,
                                    &StereoOdom::imuSensorCallback, this);  

  ROS_INFO_STREAM("StereoOdom Constructed");
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
  if (stereo_counter % 30 == 0)
  {
    ROS_INFO("VO frame received [%d]", stereo_counter);
  }

  if (!vo_core_->isPoseInitialized()){
    std::cout << "!pose_initialized. no imu input yet. will not compute VO\n";
    return;
  }
  int64_t utime_cur = (int64_t)floor(image_a_ros->header.stamp.toNSec() / 1000);
  vo_core_->setCurrentTime(utime_cur);

  int w = image_a_ros->width;
  int h = image_a_ros->height;
  int step_a = image_a_ros->step;
  int step_b = image_b_ros->step;

  // Left Image
  if (image_a_ros->encoding == "bgr8"){
    ROS_INFO_STREAM_ONCE("image_a ["<< image_a_ros->encoding <<"]. This mode has not been debugged");

    void* left_data = const_cast<void*>(static_cast<const void*>(image_a_ros->data.data()));
    memcpy(vo_core_->rgb_buf_, left_data, h*step_a);
    pixel_convert_8u_rgb_to_8u_gray(  vo_core_->left_buf_, w, w, h, vo_core_->rgb_buf_, step_a);

  } else if (image_a_ros->encoding == "rgb8"){
    ROS_INFO_STREAM_ONCE("image_a ["<< image_a_ros->encoding <<"]");
 
    // TODO: use a bgr converter as it uses a different weighting of r and b
    void* left_data = const_cast<void*>(static_cast<const void*>(image_a_ros->data.data()));
    memcpy(vo_core_->rgb_buf_, left_data, h*step_a);
    pixel_convert_8u_rgb_to_8u_gray(  vo_core_->left_buf_, w, w, h, vo_core_->rgb_buf_, step_a);

  } else if (image_a_ros->encoding == "mono8"){
    ROS_INFO_STREAM_ONCE("image_a ["<< image_a_ros->encoding <<"]");
    void* left_data = const_cast<void*>(static_cast<const void*>(image_a_ros->data.data()));
    memcpy(vo_core_->left_buf_, left_data, h*step_a);

  }else{ 
    ROS_INFO_STREAM("first image encoding not supported. something is wrong! ["<< image_a_ros->encoding <<"]");
    ROS_INFO_STREAM("Returning. not processing");
    return;
  }


  // Right Image
  ROS_INFO_STREAM_ONCE("Second Image Encoding: " << image_b_ros->encoding << " WxH: " << w << " " << h);
  if (image_b_ros->encoding == "mono16"){
    // this assumes this encoding is disparity.
    void* disp_data = const_cast<void*>(static_cast<const void*>(image_b_ros->data.data()));
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

  }else if (image_b_ros->encoding == "16UC1"){
    unsigned char* depth_data = const_cast<unsigned char*>(static_cast<const unsigned char*>(image_b_ros->data.data()));
    short* depth_short = (short*) depth_data;

    cv_bridge::CvImageConstPtr depth_img_cv;
    cv::Mat depth_mat;
    depth_img_cv = cv_bridge::toCvShare (image_b_ros, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img_cv->image.convertTo(depth_mat, CV_32F, 0.001); // NB conversion from MM to M

    // invalid RealSense depth comes as zero (0), set to NAN here
    // TODO: can this be done by directly OpenCV efficiently?
    for(int i=0; i<h*w; i++)
      if (depth_mat.data[i] == 0)
        depth_mat.data[i] = (float) NAN;

    memcpy(vo_core_->depth_buf_, depth_mat.data, h*step_b); // 2 bytes

    vo_core_->doOdometryLeftDepth();

  }else if (image_b_ros->encoding == "mono8"){
    // assumed to be grayscale from multisense
    void* right_data = const_cast<void*>(static_cast<const void*>(image_b_ros->data.data()));
    memcpy(vo_core_->right_buf_, right_data, h*step_b);
    vo_core_->doOdometryLeftRight();

  }else{ 
    ROS_INFO_STREAM("second image mode not supported. something is wrong! ["<< image_b_ros->encoding <<"]");
    ROS_INFO_STREAM("Returning. not processing");
    return;
  }


  vo_core_->doPostProcessing();

  int64_t utime_output = utime_cur;
  // use the IMU time stamp - so that the rest of the system sees a state estimate with timestamp with low latency
  if (output_using_imu_time_)
    utime_output = utime_imu_;


  tf::Transform transform;
  tf::poseEigenToTF( vo_core_->getBodyPose(), transform);
  br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(utime_output * 1E-6), "odom", "base_link"));


  geometry_msgs::PoseWithCovarianceStamped body_pose;
  body_pose.header.stamp = image_a_ros->header.stamp;
  body_pose.header.frame_id = "odom";
  geometry_msgs::Pose gpose;
  tf::poseEigenToMsg ( vo_core_->getBodyPose(), gpose);
  body_pose.pose.pose = gpose;
  body_pose_pub_.publish(body_pose);


  // This hard coded transform from base to head is because
  // when playing back logs static broadcaster doesnt work
  /*tf::Transform transform2;
  Eigen::Isometry3d null = Eigen::Isometry3d::Identity();
  null.translation().x() = 0.465;
  null.translation().y() = 0.0;
  null.translation().z() = 0.558;
  Eigen::Quaterniond quat_pitch = euler_to_quat(0, 0.28278, 0); //about 16.2 degrees
  null.rotate( quat_pitch );
  tf::poseEigenToTF(null, transform2);
  br.sendTransform(tf::StampedTransform(transform2, ros::Time().fromSec(utime_output * 1E-6), "base_link", "multisense/head"));
  */

  stereo_counter++;
}



void StereoOdom::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg)
{
  Eigen::Quaterniond imu_orientation_from_imu(msg->orientation.w,msg->orientation.x,
                                              msg->orientation.y,msg->orientation.z);
  Eigen::Vector3d gyro = Eigen::Vector3d(0,0,0); // i have nothing for this yet
  utime_imu_ = (int64_t)floor(msg->header.stamp.toNSec() / 1000);

  Eigen::Quaterniond body_orientation_from_imu = vo_core_->imuOrientationToRobotOrientation(imu_orientation_from_imu);
  vo_core_->setBodyOrientationFromImu(body_orientation_from_imu, gyro, utime_imu_);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "simple_fusion");

  FusionCoreConfig fcfg;
  fcfg.camera_config = "MULTISENSE_CAMERA";
  fcfg.output_signal = "POSE_BODY_ALT";
  fcfg.output_signal_at_10Hz = FALSE;
  fcfg.publish_feature_analysis = FALSE; 
  fcfg.fusion_mode = 0;
  fcfg.verbose = FALSE;
  fcfg.output_extension = "";
  fcfg.correction_frequency = 1;//; was typicall unused at 100;
  fcfg.feature_analysis_publish_period = 1; // 5
  std::string param_file = ""; // short filename
  fcfg.param_file = ""; // full path to file
  fcfg.draw_lcmgl = FALSE;  
  double processing_rate = 1; // real time
  fcfg.write_feature_output = FALSE;
  fcfg.which_vo_options = 2;


  ros::NodeHandle nh("~");
  nh.getParam("verbose", fcfg.verbose);
  nh.getParam("extrapolate_when_vo_fails", fcfg.extrapolate_when_vo_fails);
  nh.getParam("draw_lcmgl", fcfg.draw_lcmgl);
  nh.getParam("publish_feature_analysis", fcfg.publish_feature_analysis);
  nh.getParam("param_file", param_file); // short filename
  nh.getParam("output_body_pose_lcm", fcfg.output_signal);
  nh.getParam("which_vo_options", fcfg.which_vo_options);
  nh.getParam("fusion_mode", fcfg.fusion_mode);
  nh.getParam("camera_config", fcfg.camera_config);

  char* drs_base;
  drs_base = getenv ("DRS_BASE");

  std::string configPath;
  configPath = std::string( getenv ("DRS_BASE") ) + "/params/config";

  std::cout << configPath << "\n";
  

  fcfg.param_file = configPath +'/' + std::string(param_file);
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



  cout << fcfg.fusion_mode << " is fusion_mode\n";
  cout << fcfg.camera_config << " is camera_config\n";
  cout << fcfg.param_file << " is param_file [full path]\n";



  new StereoOdom(nh, lcm_recv, lcm_pub, fcfg);
  ros::spin();
  //while(0 == lcm_recv->handle());
}
