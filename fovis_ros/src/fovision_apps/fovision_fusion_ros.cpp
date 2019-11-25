#include "fovision_apps/stereo_odom_ros.hpp"


using namespace std;


int main(int argc, char **argv){
  ros::init(argc, argv, "simple_fusion");
  ros::NodeHandle nh("~");

  FusionCoreConfig fcfg;
  fcfg.config_filename = "anymal.yaml";
  nh.getParam("verbose", fcfg.verbose);
  nh.getParam("extrapolate_when_vo_fails", fcfg.extrapolate_when_vo_fails);
  nh.getParam("publish_feature_analysis", fcfg.publish_feature_analysis);

  nh.getParam("which_vo_options", fcfg.which_vo_options);
  cout << fcfg.which_vo_options << " is which_vo_options\n";

  nh.getParam("orientation_fusion_mode", fcfg.orientation_fusion_mode);
  cout << fcfg.orientation_fusion_mode << " is orientation_fusion_mode\n";

  nh.getParam("pose_initialization_mode", fcfg.pose_initialization_mode);
  cout << fcfg.pose_initialization_mode << " is pose_initialization_mode\n";

  nh.getParam("config_filename", fcfg.config_filename);
  cout << fcfg.config_filename << " is config_filename [full path]\n";

  StereoOdomConfig socfg;
  socfg.output_tf_frame = "base_fovis";

  nh.getParam("output_tf_frame", socfg.output_tf_frame);
  cout << "output_tf_frame: publish odom to "<< socfg.output_tf_frame << "\n";

  socfg.write_pose_to_file = false;
  nh.getParam("write_pose_to_file", socfg.write_pose_to_file);
  cout << "Write pose to file: " << boolalpha << socfg.write_pose_to_file << endl;



  StereoOdom so(nh, socfg, fcfg);
  ros::spin();
}
