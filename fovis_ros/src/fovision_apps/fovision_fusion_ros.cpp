// A VO-based non-probablistic state estimator for the multisense
// - occasionally uses IMU to avoid orientation drift
// - when VO fails extrapolate using previous vision lin rate and imu rot rates

// For IMU orientation integration:
// Estimate is maintained in the body frame which is assumed to be
// Forward-Left-Up such at roll and pitch can be isolated from yaw.

#include <fovision_apps/stereo_odom_ros.hpp>


using namespace std;


int main(int argc, char **argv){
  ros::init(argc, argv, "simple_fusion");

  FusionCoreConfig fcfg;
  fcfg.output_extension = "";
  fcfg.correction_frequency = 1;//; was typicall unused at 100;
  fcfg.feature_analysis_publish_period = 1; // 5
  double processing_rate = 1; // real time
  fcfg.write_feature_output = false;

  fcfg.verbose = false;
  fcfg.extrapolate_when_vo_fails = false;
  fcfg.publish_feature_analysis = false; 
  fcfg.output_signal = "POSE_BODY_ALT";
  fcfg.output_tf_frame = "/fovis/base";
  fcfg.which_vo_options = 2;
  fcfg.orientation_fusion_mode = 0;
  fcfg.pose_initialization_mode = 0;
  fcfg.camera_config = "MULTISENSE_CAMERA";
  fcfg.config_filename = "anymal.yaml";
  fcfg.write_pose_to_file = false;


  ros::NodeHandle nh("~");
  nh.getParam("verbose", fcfg.verbose);
  nh.getParam("extrapolate_when_vo_fails", fcfg.extrapolate_when_vo_fails);
  nh.getParam("publish_feature_analysis", fcfg.publish_feature_analysis);
  nh.getParam("output_body_pose_lcm", fcfg.output_signal);
  nh.getParam("output_tf_frame", fcfg.output_tf_frame);
  nh.getParam("which_vo_options", fcfg.which_vo_options);
  nh.getParam("orientation_fusion_mode", fcfg.orientation_fusion_mode);
  nh.getParam("pose_initialization_mode", fcfg.pose_initialization_mode);
  nh.getParam("camera_config", fcfg.camera_config);
  nh.getParam("config_filename", fcfg.config_filename);
  nh.getParam("write_pose_to_file", fcfg.write_pose_to_file);

  char* drs_base;
  drs_base = getenv ("DRS_BASE");

  std::string configPath;
  configPath = std::string( getenv ("DRS_BASE") ) + "/config";

  std::cout << configPath << "\n";
  


  cout << fcfg.orientation_fusion_mode << " is orientation_fusion_mode\n";
  cout << fcfg.pose_initialization_mode << " is pose_initialization_mode\n";
  cout << fcfg.camera_config << " is camera_config\n";
  cout << "output_tf_frame: publish odom to "<< fcfg.output_tf_frame << "\n";
  cout << fcfg.config_filename << " is config_filename [full path]\n";
  cout << fcfg.which_vo_options << " is which_vo_options\n";



  StereoOdom* so = new StereoOdom(nh, fcfg);
  ros::spin();
  delete so;
}
