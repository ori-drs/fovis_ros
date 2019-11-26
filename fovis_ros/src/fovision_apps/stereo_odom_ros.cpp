#include "fovision_apps/stereo_odom_ros.hpp"
#include <fovis_msgs/Stats.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
using namespace cv; // for disparity ops

StereoOdom::StereoOdom(ros::NodeHandle& node_in,
                       const StereoOdomConfig& cfg,
                       const FusionCoreConfig& fcfg) :
       node_(node_in), it_(node_), sync_(10), sync_without_info_(10),
       cfg_(cfg), fcfg_(fcfg), vo_core_(new FusionCore(fcfg_))
{

  // Parameters:
  output_using_imu_time_ = true;
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

  image_a_ros_sub_.subscribe(it_, image_a_string, 30, image_transport::TransportHints( image_a_transport ));
  image_b_ros_sub_.subscribe(it_, image_b_string, 30, image_transport::TransportHints( image_b_transport ));

  //info_a_ros_sub_.subscribe(node_, ros::names::resolve(info_a_string), 30);
  //info_b_ros_sub_.subscribe(node_, ros::names::resolve(info_b_string), 30);
  //sync_.connectInput(image_a_ros_sub_, info_a_ros_sub_, image_b_ros_sub_, info_b_ros_sub_);
  //sync_.registerCallback(boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4));
  sync_without_info_.connectInput(image_a_ros_sub_, image_b_ros_sub_);
  sync_without_info_.registerCallback(&StereoOdom::stereoCallback, this);

  std::string input_imu_topic;
  node_.getParam("input_imu_topic", input_imu_topic);
  imuSensorSub_ = node_.subscribe(input_imu_topic, 100,
                                  &StereoOdom::imuSensorCallback, this);

  std::string input_body_pose_topic;
  node_.getParam("input_body_pose_topic", input_body_pose_topic);
  poseOdomSub_ = node_.subscribe(input_body_pose_topic, 100,
                                    &StereoOdom::poseOdomCallback, this);

  std::string output_body_pose_topic;
  node_.getParam("output_body_pose_topic", output_body_pose_topic);
  body_pose_pub_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(output_body_pose_topic, 10);

  fovis_stats_pub_ = node_.advertise<fovis_msgs::Stats>("/fovis/stats", 10);
  delta_vo_pub_ = node_.advertise<fovis_msgs::VisualOdometryUpdate>("/fovis/delta_vo", 10);

  if (fcfg.publish_feature_analysis) {
    features_image_pub_ = node_.advertise<sensor_msgs::Image>("/fovis/features_image", 1);
    features_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/fovis/features_cloud", 1);
  }

  ROS_INFO_STREAM("StereoOdom Constructed");
}


void StereoOdom::stereoWithInfoCallback(const sensor_msgs::ImageConstPtr& image_a_ros,
                                        const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                                        const sensor_msgs::ImageConstPtr& image_b_ros,
                                        const sensor_msgs::CameraInfoConstPtr& info_b_ros)
{

}

void StereoOdom::publishDeltaVO() {
  uint64_t prev_timestamp;
  uint64_t curr_timestamp;
  Eigen::Isometry3d relative_pose;

  // don't publish if the retrieval fails
  if(!vo_core_->getBodyRelativePose(prev_timestamp,
                                    curr_timestamp,
                                    relative_pose))
  {
    return;
  }

  delta_vo_msg_.header.stamp = ros::Time::now();
  delta_vo_msg_.prev_timestamp = ros::Time().fromNSec(prev_timestamp * 1e3);
  delta_vo_msg_.curr_timestamp = ros::Time().fromNSec(curr_timestamp * 1e3);
  delta_vo_msg_.covariance.fill(0);
  geometry_msgs::Transform t;
  tf::transformEigenToMsg(relative_pose,t);
  delta_vo_msg_.relative_transform = t;
  delta_vo_msg_.estimate_status = vo_core_->getVisualOdometry()->getMotionEstimateStatus();
  delta_vo_pub_.publish(delta_vo_msg_);
}


int StereoOdom::convertPixelRGBtoGray (uint8_t *dest,
                                       int dstride,
                                       int width,
                                       int height,
                                       const uint8_t *src,
                                       int sstride)
{
  int i, j;
  for (i = 0; i < height; i++) {
    uint8_t *drow = dest + i * dstride;
    const uint8_t *srow = src + i * sstride;
    for (j = 0; j < width; j++) {
      drow[j] = 0.2125 * srow[j*3+0] + 0.7154 * srow[j*3+1] + 0.0721 * srow[j*3+2];
    }
  }
  return 0;
}


void StereoOdom::publishFovisStats(int sec, int nsec){
  const fovis::VisualOdometry* odom = vo_core_->getVisualOdometry();
  const fovis::MotionEstimator* me = odom->getMotionEstimator();
  fovis_msgs::Stats stats;
  stats.header.stamp.sec = sec;
  stats.header.stamp.nsec = nsec;
  stats.num_matches = me->getNumMatches();
  stats.num_inliers = me->getNumInliers();
  stats.mean_reprojection_error = me->getMeanInlierReprojectionError();
  stats.num_reprojection_failures = me->getNumReprojectionFailures();

  const fovis::OdometryFrame * tf(odom->getTargetFrame());
  stats.num_detected_keypoints = tf->getNumDetectedKeypoints();
  stats.num_keypoints = tf->getNumKeypoints();
  stats.fast_threshold = odom->getFastThreshold();

  fovis::MotionEstimateStatusCode estim_status = odom->getMotionEstimateStatus();
  stats.status = (int) estim_status;

  fovis_stats_pub_.publish(stats);
}


void StereoOdom::stereoCallback(const sensor_msgs::ImageConstPtr& image_a_ros,
                                const sensor_msgs::ImageConstPtr& image_b_ros)
{
  if (stereo_counter % 120 == 0)
  {
    ROS_INFO("VO frame received [%d]", stereo_counter);
  }

  if (!vo_core_->isPoseInitialized()){
    std::cout << "pose not initialized. Will not compute VO\n";
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
    convertPixelRGBtoGray(vo_core_->left_buf_, w, w, h, vo_core_->rgb_buf_, step_a);

  } else if (image_a_ros->encoding == "rgb8"){
    ROS_INFO_STREAM_ONCE("image_a ["<< image_a_ros->encoding <<"]");

    // TODO: use a bgr converter as it uses a different weighting of r and b
    void* left_data = const_cast<void*>(static_cast<const void*>(image_a_ros->data.data()));
    memcpy(vo_core_->rgb_buf_, left_data, h*step_a);
    convertPixelRGBtoGray(vo_core_->left_buf_, w, w, h, vo_core_->rgb_buf_, step_a);

  } else if (image_a_ros->encoding == "mono8" || image_a_ros->encoding == "8UC1")
  {
    // assumed to be grey scale
    ROS_INFO_STREAM_ONCE("image_a ["<< image_a_ros->encoding <<"]");
    void* left_data = const_cast<void*>(static_cast<const void*>(image_a_ros->data.data()));
    memcpy(vo_core_->left_buf_, left_data, h*step_a);

  } else{
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

    // Currently disabled but worked previously (in LCM)
    //if ( vo_core_->isFilterDisparityEnabled() ){
    //  // Filter the data to remove far away depth and the crash bar (from hyq)
    //  vo_core_->filterDisparity(w, h);
    //}

    vo_core_->doOdometryLeftDisparity();

  } else if (image_b_ros->encoding == "16UC1"){
    unsigned char* depth_data = const_cast<unsigned char*>(static_cast<const unsigned char*>(image_b_ros->data.data()));
    short* depth_short = (short*) depth_data;

    cv_bridge::CvImageConstPtr depth_img_cv;
    cv::Mat depth_mat;
    depth_img_cv = cv_bridge::toCvShare (image_b_ros, sensor_msgs::image_encodings::TYPE_16UC1);
    depth_img_cv->image.convertTo(depth_mat, CV_32F, 0.001); // NB conversion from MM to M

    memcpy(vo_core_->depth_buf_, depth_mat.data, h*w*4); // 4 bytes per float

    // invalid RealSense depth comes as zero (0), set to NAN here
    // TODO: can this be done more efficiently?
    if ( vo_core_->isFilterDepthEnabled() ){
      // Filter the data to remove far away depth and the crash bar (from hyq)
      vo_core_->filterDepth(w, h);
    }

    vo_core_->doOdometryLeftDepth();

  }else if (image_b_ros->encoding == "mono8" || image_b_ros->encoding == "8UC1")
  {
    // assumed to be grayscale from multisense
    // this works with realsense d435 ir also
    void* right_data = const_cast<void*>(static_cast<const void*>(image_b_ros->data.data()));
    memcpy(vo_core_->right_buf_, right_data, h*step_b);
    vo_core_->doOdometryLeftRight();
  } else{
    ROS_INFO_STREAM("second image mode not supported. something is wrong! ["<< image_b_ros->encoding <<"]");
    ROS_INFO_STREAM("Returning. not processing");
    return;
  }

  // This is where inertial data is fused
  vo_core_->doPostProcessing();
  publishDeltaVO();

  publishFovisStats(image_a_ros->header.stamp.sec, image_a_ros->header.stamp.nsec);

  if (fcfg_.publish_feature_analysis){
    uint8_t* feature_image_ = vo_core_->getFeaturesImage();
    cv::Mat A(h,w,CV_8UC1);
    A.data = feature_image_;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", A).toImageMsg();
    features_image_pub_.publish(msg);

    pcl::PointCloud<pcl::PointXYZRGB> features_cloud = vo_core_->getFeaturesCloud();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(features_cloud, cloud_msg);
    cloud_msg.header.stamp = image_a_ros->header.stamp;
    cloud_msg.header.frame_id = cfg_.output_tf_frame;//image_a_ros->header.frame_id;
    features_cloud_pub_.publish(cloud_msg);
  }

  int64_t utime_output = utime_cur;
  // use the IMU time stamp - so that the rest of the system sees a state estimate with timestamp with low latency
  if (output_using_imu_time_){
    utime_output = utime_imu_;
  }

  tf::Transform transform;
  tf::poseEigenToTF(vo_core_->getBodyPose(), transform);

  br.sendTransform(tf::StampedTransform(transform, ros::Time().fromNSec(utime_output * 1e3), "odom", cfg_.output_tf_frame));


  geometry_msgs::PoseWithCovarianceStamped body_pose;
  body_pose.header.stamp = image_a_ros->header.stamp;
  body_pose.header.frame_id = "odom";
  geometry_msgs::Pose gpose;
  tf::poseEigenToMsg(vo_core_->getBodyPose(), gpose);
  body_pose.pose.pose = gpose;
  body_pose_pub_.publish(body_pose);

  if (cfg_.write_pose_to_file){
    vo_core_->writePoseToFile(vo_core_->getBodyPose(), utime_output);
  }

  stereo_counter++;
}



void StereoOdom::imuSensorCallback(const sensor_msgs::ImuConstPtr& msg)
{
  Eigen::Quaterniond imu_orientation_from_imu(msg->orientation.w,
                                              msg->orientation.x,
                                              msg->orientation.y,
                                              msg->orientation.z);

  Eigen::Vector3d gyro = Eigen::Vector3d(0,0,0); // i have nothing for this yet
  utime_imu_ = (int64_t)floor(msg->header.stamp.toNSec() / 1000);

  Eigen::Quaterniond body_orientation_from_imu = vo_core_->imuOrientationToRobotOrientation(imu_orientation_from_imu);

  if ((fcfg_.orientation_fusion_mode==1) ||  (fcfg_.orientation_fusion_mode == 2) ){
    vo_core_->setBodyOrientationFromImu(body_orientation_from_imu, gyro, utime_imu_);
  }


  if ( (!vo_core_->isPoseInitialized()) &&  (fcfg_.pose_initialization_mode == 1) ){
    std::cout << "IMU callback: initializing pose using IMU (roll and pitch)\n";
    Eigen::Isometry3d init_pose;
    init_pose.setIdentity();
    init_pose.translation() << 0,0,0;

    double rpy_imu[3];
    quat_to_euler(  body_orientation_from_imu , rpy_imu[0], rpy_imu[1], rpy_imu[2]);
    init_pose.rotate( euler_to_quat( rpy_imu[0], rpy_imu[1], 0) ); // not using yaw
    init_pose.rotate(body_orientation_from_imu);
    vo_core_->initializePose(init_pose);
  }
}


void StereoOdom::poseOdomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{

  Eigen::Quaterniond body_orientation_from_odom(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,
                                              msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
  Eigen::Vector3d gyro = Eigen::Vector3d(0,0,0); // i have nothing for this yet
  utime_imu_ = (int64_t)floor(msg->header.stamp.toNSec() / 1000);

  if ((fcfg_.orientation_fusion_mode==3) ||  (fcfg_.orientation_fusion_mode ==4)  ){
    vo_core_->setBodyOrientationFromImu(body_orientation_from_odom, gyro, utime_imu_);
  }



  if (!vo_core_->isPoseInitialized()){
    std::cout << "Pose Odom callback: initializing pose using IMU\n";
    Eigen::Isometry3d init_pose = Eigen::Isometry3d::Identity();
    tf::poseMsgToEigen(msg->pose.pose, init_pose);
    vo_core_->initializePose(init_pose);
  }
}
