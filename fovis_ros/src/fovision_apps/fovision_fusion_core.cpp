#include "voconfig/voconfig.hpp"
#include "vofeatures/vofeatures.hpp"
#include "voestimator/voestimator.hpp"
#include "fovision/fovision.hpp"

#include "fovision_apps/fovision_fusion_core.hpp"

#include <opencv/cv.h> // for disparity 

using namespace std;
using namespace cv; // for disparity ops


std::ofstream fovision_output_file_;

FusionCore::FusionCore(const FusionCoreConfig& fcfg) :
       fcfg_(fcfg), utime_cur_(0), utime_prev_(0),
       ref_utime_(0), changed_ref_frames_(false)
{

  config_ = new voconfig::FovisYAMLConfigurator(fcfg_.config_filename);
  std::shared_ptr<fovis::StereoCalibration> stereo_calibration_(config_->loadStereoCalibration());

  // Disparity filtering
  //filter_disparity_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.filter.enabled");
  std::cout << "Disparity Filter is " << (filter_disparity_ ? "ENABLED" : "DISABLED")  << "\n";
  //filter_disparity_below_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_disparity_below_threshold");
  //filter_disparity_above_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_disparity_above_threshold");

  // Depth Filtering
  //filter_depth_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.filter.filter_depth_enabled");
  std::cout << "Depth Filter is " << (filter_depth_ ? "ENABLED" : "DISABLED")  << "\n";
  //filter_depth_below_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_depth_below_threshold");
  //filter_depth_above_threshold_ = bot_param_get_double_or_fail(botparam_, "visual_odometry.filter.filter_depth_above_threshold");


  //filter_image_rows_above_ = bot_param_get_int_or_fail(botparam_, "visual_odometry.filter.filter_image_rows_above");
  //publish_filtered_image_ = bot_param_get_boolean_or_fail(botparam_, "visual_odometry.filter.publish_filtered_image");

  // Allocate various buffers:
  image_size_ = stereo_calibration_->getWidth() * stereo_calibration_->getHeight();
  left_buf_ = (uint8_t*) malloc(3*image_size_);
  right_buf_ = (uint8_t*) malloc(3*image_size_);
  left_buf_ref_ = (uint8_t*) malloc(3*image_size_); // used of feature output 
  rgb_buf_ = (uint8_t*) malloc(10*image_size_ * sizeof(uint8_t)); 
  decompress_disparity_buf_ = (uint8_t*) malloc( 4*image_size_*sizeof(uint8_t));  // arbitary size chosen..
  depth_buf_ = (float*) malloc( image_size_*sizeof(float));  // byte per image

  vo_ = new FoVision(stereo_calibration_, fcfg_.which_vo_options);
  vo_->setPublishFovisStats(fcfg_.publish_feature_analysis);

  Eigen::Isometry3d body_to_camera_ = config_->B_t_BC();

  features_ = new VoFeatures(stereo_calibration_->getWidth(),
                             stereo_calibration_->getHeight(),
                             body_to_camera_);
  estimator_ = new VoEstimator();


  estimator_->setCameraToBody(body_to_camera_.inverse());


  pose_initialized_ = false;
  // if not using imu or pose, initialise with robot model
  if (fcfg_.pose_initialization_mode == 0){
    std::cout << "Pose initialized using cfg\n";
    Eigen::Isometry3d body_to_local_initial = Eigen::Isometry3d::Identity();
    //get_trans_with_utime( botframes_ ,  "body", "local", 0, body_to_local_initial);
    estimator_->setBodyPose(body_to_local_initial);  
    pose_initialized_ = true;
  }


  // IMU:
  local_to_body_orientation_from_imu_initialized_ = false;
  local_to_body_orientation_from_imu_ = Eigen::Quaterniond(1,0,0,0);
  imu_counter_ = 0;

  body_to_imu_ = Eigen::Isometry3d::Identity();
  imu_to_camera_ = config_->B_t_BC();

  cout << "FusionCore Constructed"  << endl;
}

FusionCore::~FusionCore(){
      free (left_buf_);
      free(right_buf_);
      free(depth_buf_);
      free(rgb_buf_);
      free(decompress_disparity_buf_);
      delete vo_;
      delete features_;
      delete estimator_;
      free(left_buf_ref_);
}


void FusionCore::featureAnalysis(){

  /// Incremental Feature Output:
  if (counter % fcfg_.feature_analysis_publish_period == 0 ){
    features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , utime_cur_);
    features_->setCurrentImage(left_buf_);
    features_->setCurrentCameraPose( estimator_->getCameraPose() );
    if (vo_->getNumMatches()>0)
      features_->doFeatureProcessing(true, fcfg_.write_feature_output ); // use current features
    else
      std::cout << "no matches, will not publish or output\n"; // this happens for first frame
  }
  
  /// Reference Feature Output: ///////////////////////////////////////////////
  // Check we changed reference frame last iteration, if so output the set of matching inliers:
  if (changed_ref_frames_) {
    if (ref_utime_ > 0){ // skip the first null image
      if(vo_->getNumMatches() > 200){ // if less than 50 features - dont bother writing
      // was:      if(featuresA.size() > 50){ // if less than 50 features - dont bother writing
        cout << "changed ref frame from " << utime_prev_ << " at " << utime_cur_ <<  " with " <<vo_->getNumMatches()<<" matches\n";
        features_->setFeatures(vo_->getMatches(), vo_->getNumMatches() , ref_utime_);
        features_->setReferenceImage(left_buf_ref_);
        features_->setReferenceCameraPose( ref_camera_pose_ );
        features_->doFeatureProcessing(false, fcfg_.write_feature_output ); // use reference features
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


void FusionCore::updateMotion() {
  // 1. Estimate the motion using VO
  Eigen::Isometry3d delta_camera;
  Eigen::MatrixXd delta_cov;
  fovis::MotionEstimateStatusCode delta_status;

  vo_->getMotion(delta_camera, delta_cov, delta_status );

  // Try to catch all NaNs output by Fovis by exiting
  if (std::isnan(delta_camera.translation().x()) ){
    std::cout << utime_cur_ << ": got nan\n";
    delta_status = fovis::REPROJECTION_ERROR; // not success
    exit(-1);
  }

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

  } else if (fcfg_.extrapolate_when_vo_fails){
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
      Eigen::Quaterniond extrapolated_quat;
      extrapolated_quat = euler_to_quat( camera_angular_velocity_from_imu_[0]*dt, camera_angular_velocity_from_imu_[1]*dt, camera_angular_velocity_from_imu_[2]*dt);

      delta_camera.setIdentity();
      delta_camera.translation().x() = vo_velocity_linear_[0] * dt;
      delta_camera.translation().y() = vo_velocity_linear_[1] * dt;
      delta_camera.translation().z() = vo_velocity_linear_[2] * dt;
      delta_camera.rotate(extrapolated_quat);
    }
  } else {
    //std::cout << "VO failed. Not extrapolating. output identity "<< utime_cur_ <<"\n";
    delta_camera.setIdentity();
  }

  if (fcfg_.verbose){
    std::stringstream ss2;
    //print_Isometry3d(delta_camera, ss2);
    std::cout << "camera: " << ss2.str() << " code: " << (int) delta_status << "\n";
  }

  // 3. Update the motion estimation:
  estimator_->updatePose(utime_cur_, utime_prev_, delta_camera);

}


void FusionCore::filterDisparity(int w, int h){
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
}

void FusionCore::filterDepth(int w, int h){

  for(int i=0; i<h*w; i++){
    if (depth_buf_[i] < filter_depth_below_threshold_)
      depth_buf_[i] = NAN;
    if (depth_buf_[i] > filter_depth_above_threshold_)
      depth_buf_[i] = NAN;
  }

}


// Transform the Microstrain IMU orientation into the body frame:
Eigen::Quaterniond FusionCore::imuOrientationToRobotOrientation(const Eigen::Quaterniond& imu_orientation_from_imu){
  Eigen::Isometry3d motion_estimate;
  motion_estimate.setIdentity();
  motion_estimate.translation() << 0,0,0;
  motion_estimate.rotate(imu_orientation_from_imu);

  // rotate IMU orientation so that look vector is +X, and up is +Z
  // TODO: do this with rotation matrices for efficiency:
  Eigen::Isometry3d body_pose_from_imu = motion_estimate * body_to_imu_;
  Eigen::Quaterniond body_orientation_from_imu(body_pose_from_imu.rotation());

  return body_orientation_from_imu;
}





void FusionCore::fuseInertial(const Eigen::Quaterniond& local_to_body_orientation_from_imu,
                              int64_t utime)
{
  if (fcfg_.orientation_fusion_mode==0){ // Ignore any imu or pose orientation measurements
    // Publish the pose
  }else{
    if (imu_counter_== fcfg_.correction_frequency){
      // Every X frames: replace the pitch and roll with that from the IMU
      // convert the camera pose to head frame
      // extract xyz and yaw from head frame
      // extract pitch and roll from imu (in head frame)
      // combine, convert to camera frame... set as pose

      // 1. Get the currently estimated head pose and its rpy
      Eigen::Isometry3d local_to_body = estimator_->getBodyPose();
      std::stringstream ss2;
      //print_Isometry3d(local_to_body, ss2);
      double rpy[3];
      quat_to_euler(  Eigen::Quaterniond(local_to_body.rotation()) , rpy[0], rpy[1], rpy[2]);
      if (fcfg_.verbose){
        std::cout << "local_to_body: " << ss2.str() << " | "<< 
          rpy[0]*180/M_PI << " " << rpy[1]*180/M_PI << " " << rpy[2]*180/M_PI << "\n";        
      }
        
      // 2. Get the IMU orientated RPY:
      double rpy_imu[3];
      quat_to_euler( local_to_body_orientation_from_imu , 
                      rpy_imu[0], rpy_imu[1], rpy_imu[2]);
      if (fcfg_.verbose){
        std::cout <<  rpy_imu[0]*180/M_PI << " " << rpy_imu[1]*180/M_PI << " " << rpy_imu[2]*180/M_PI << " rpy_imu\n";        
        cout << "IMU correction | roll pitch | was: "
            << rpy[0]*180/M_PI << " " << rpy[1]*180/M_PI << " | now: "
            << rpy_imu[0]*180/M_PI << " " << rpy_imu[1]*180/M_PI << "\n";
      }
      
      // 3. Merge the two orientation estimates:
      Eigen::Quaterniond revised_local_to_body_quat;
      if ( (fcfg_.orientation_fusion_mode==1) || (fcfg_.orientation_fusion_mode==3)){
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
      if (fcfg_.verbose){
        std::stringstream ss4;
        //print_Isometry3d(revised_local_to_body, ss4);
        quat_to_euler(  Eigen::Quaterniond(revised_local_to_body.rotation()) , rpy[0], rpy[1], rpy[2]);
        std::cout << "local_revhead: " << ss4.str() << " | "<< 
          rpy[0]*180/M_PI << " " << rpy[1]*180/M_PI << " " << rpy[2]*180/M_PI << "\n";        
      }
      estimator_->setBodyPose(revised_local_to_body);
    }
    if (imu_counter_ > fcfg_.correction_frequency) { imu_counter_ =0; }
    imu_counter_++;
  }

}


void FusionCore::writePoseToFile(const Eigen::Isometry3d& pose, int64_t utime){

  if(!fovision_output_file_.is_open()){
    std::cout << "pose_body_trajectory.txt file not open, opening it\n";
    fovision_output_file_.open(  "pose_body_trajectory.txt" , std::fstream::out);
    fovision_output_file_ << "# utime x y z qw qx qy qz roll pitch yaw\n";
    fovision_output_file_.flush();
  }

  // Publish the current estimate

  double current_rpy[3];
  Eigen::Quaterniond current_quat = Eigen::Quaterniond( pose.rotation() );
  quat_to_euler( current_quat, current_rpy[0], current_rpy[1], current_rpy[2]);

  if(fovision_output_file_.is_open()){
    fovision_output_file_ << utime  << ","
                  << pose.translation().x() << "," << pose.translation().y() << "," << pose.translation().z() << ","
                  << current_quat.w() << "," << current_quat.x() << "," << current_quat.y() << "," << current_quat.z() << ","
                  << current_rpy[0] << "," << current_rpy[1] << "," << current_rpy[2] << "\n";
    fovision_output_file_.flush();
  }else{
    std::cout << "pose_body_trajectory.txt not open still\n";
  }
}


void FusionCore::setBodyOrientationFromImu(const Eigen::Quaterniond& local_to_body_orientation_from_imu,
                                           const Eigen::Vector3d& gyro,
                                           int64_t imu_utime)
{
  local_to_body_orientation_from_imu_ = local_to_body_orientation_from_imu;
  local_to_body_orientation_from_imu_initialized_ = true;

  // Transform rotation Rates into body frame:
  double camera_ang_vel_from_imu_[3];
  Eigen::Quaterniond imu_to_camera_quat = Eigen::Quaterniond( imu_to_camera_.rotation() );
  double imu_to_camera_quat_array[4];
  imu_to_camera_quat_array[0] =imu_to_camera_quat.w(); imu_to_camera_quat_array[1] =imu_to_camera_quat.x();
  imu_to_camera_quat_array[2] =imu_to_camera_quat.y(); imu_to_camera_quat_array[3] =imu_to_camera_quat.z();
  double gyro_array[3];
  gyro_array[0] = gyro[0]; gyro_array[1] = gyro[1]; gyro_array[2] = gyro[2];
  quat_rotate_to( imu_to_camera_quat_array, gyro_array, camera_ang_vel_from_imu_);
  camera_angular_velocity_from_imu_ = Eigen::Vector3d(camera_ang_vel_from_imu_[0], camera_ang_vel_from_imu_[1], camera_ang_vel_from_imu_[2]);
  // Didn't find this necessary - for smooth motion
  //camera_angular_velocity_from_imu_alpha_ = 0.8*camera_angular_velocity_from_imu_alpha_ + 0.2*camera_angular_velocity_from_imu_;

  camera_linear_velocity_from_imu_  = Eigen::Vector3d(0,0,0);

  // experimentally correct for sensor timing offset:
  //int64_t temp_utime = imu_utime;// + 120000;
  //estimator_->publishPose(temp_utime, "POSE_IMU_RATES", Eigen::Isometry3d::Identity(), camera_linear_velocity_from_imu_, camera_angular_velocity_from_imu_);
  // estimator_->publishPose(temp_utime, "POSE_IMU_RATES", Eigen::Isometry3d::Identity(), camera_linear_velocity_from_imu_, camera_angular_velocity_from_imu_alpha_);

}
