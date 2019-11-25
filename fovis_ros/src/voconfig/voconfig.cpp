/*
 * configuration.cpp
 *
 *  Created on: Dec 5, 2011
 *      Author: Maurice Fallon
 *      Author: Hordur Johannsson
 */

#include "voconfig/voconfig.hpp"
#include <iostream>
#include <boost/lexical_cast.hpp>
#include <Eigen/Dense>

#include <opencv2/core.hpp>
#include <eigen_utils/eigen_utils.hpp>

/// \brief Parse the contents of a 3 element vector.
Eigen::Vector3d parseVector3d(const cv::FileNode& node) {
  if (node.size()!=3) {
    //std::cerr << "Using defalt value for vector.." << std::endl;
    return Eigen::Vector3d{0,0,0};
  }
  return Eigen::Vector3d{static_cast<double>(node[0]),
                         static_cast<double>(node[1]),
                         static_cast<double>(node[2])};
}

namespace voconfig
{

BotParamConfiguration::BotParamConfiguration(const std::string & key_prefix) : key_prefix_(key_prefix)
{
}

BotParamConfiguration::~BotParamConfiguration() {}

bool BotParamConfiguration::has_key(const std::string & key)
{
  return false;
}

bool BotParamConfiguration::get(const std::string & key, bool default_value)
{
  int bval;
  return bval;
}

int BotParamConfiguration::get(const std::string & key, int default_value)
{
  int ival;
  return ival;
}

double BotParamConfiguration::get(const std::string & key, double default_value)
{
  double dval;
  return dval;
}

std::string BotParamConfiguration::get(const std::string & key, const std::string & default_value)
{
  std::string val;
  return val;
}

KmclConfiguration::KmclConfiguration(const std::string & config_filename)
    : depth_source_type_(UNKNOWN), config_filename_(config_filename)
{
  init();
}

void KmclConfiguration::init() {
  
  // new
  depth_source_type_ = STEREO;

  cv::FileStorage file(config_filename_, cv::FileStorage::READ);
  if (!file.isOpened()) {
    std::cout << "could not read the yaml config file\n";
    exit(-1);
  }

  double stereo_baseline = static_cast<double>(file["cameras"][0]["stereo_baseline"]); // actual value should be negative e.g. -0.05
  std::cout << stereo_baseline << " \n";

  Eigen::Vector3d B_r_BC = parseVector3d(file["cameras"][0]["B_r_BC"]);

  Eigen::Quaterniond B_q_BC;
  B_q_BC.w() = static_cast<double>(file["cameras"][0]["B_q_BC"][3]);
  B_q_BC.x() = static_cast<double>(file["cameras"][0]["B_q_BC"][0]);
  B_q_BC.y() = static_cast<double>(file["cameras"][0]["B_q_BC"][1]);
  B_q_BC.z() = static_cast<double>(file["cameras"][0]["B_q_BC"][2]);

  std::cout << "Received Body to Camera rotation [RPY, deg]: " << eigen_utils::getEulerAnglesDeg(B_q_BC).transpose() << std::endl;
  // normalize the quaternion we read from config file to be sure it's valid
  B_q_BC.normalize();
  // Eigen:Quaternion is WXYZ | yaml is XYZW

  B_t_BC_ = Eigen::Isometry3d::Identity();
  B_t_BC_.translation() = B_r_BC;
  B_t_BC_.rotate( B_q_BC );

  std::cout << B_r_BC << " [B_r_BC]\n";
  std::cout << B_q_BC.x() << " " << B_q_BC.y() << " " << B_q_BC.z() << " " << B_q_BC.w() << " B_q_BC [xyzw]\n";

  Eigen::Quaterniond B_q_BC2 = Eigen::Quaterniond(B_t_BC_.rotation());
  std::cout << B_q_BC2.x() << " " << B_q_BC2.y() << " " << B_q_BC2.z() << " " << B_q_BC2.w() << " B_q_BC [xyzw]\n";

  // cameras
  /*
  key_prefix_ = "cameras." + depth_source_name;

  char * depth_source_type_str = bot_param_get_str_or_fail(bot_param_, (key_prefix_ + ".type").c_str());
  if (depth_source_type_str == std::string("stereo")) {
    depth_source_type_ = STEREO;
  } else if (depth_source_type_str == std::string("openni")) {
    depth_source_type_ = OPENNI;
  } else if (depth_source_type_str == std::string("primesense")) {
    depth_source_type_ = PRIMESENSE;
  } else {
    std::cerr << "Unknown depth source type: '" << depth_source_type_str
              << "'. Should be 'stereo' or 'primesense'\n";
    exit(-1);
  }
  free (depth_source_type_str);
  */
}

KmclConfiguration::~KmclConfiguration()
{
}

std::shared_ptr<fovis::PrimeSenseCalibration>
KmclConfiguration::load_primesense_calibration() const {
  assert (depth_source_type_==PRIMESENSE || depth_source_type_==OPENNI);
  if (depth_source_type_ != PRIMESENSE && depth_source_type_ != OPENNI) {
    return std::shared_ptr<fovis::PrimeSenseCalibration>();
  }
  std::string key_prefix_str;
  fovis::PrimeSenseCalibrationParameters kparams;
  for (int i=0; i < 2; ++i) {
    fovis::CameraIntrinsicsParameters* params;

    if (i == 0) {
      key_prefix_str = std::string(key_prefix_) + ".rgb";
      params = &(kparams.rgb_params);
    } else {
      key_prefix_str = std::string(key_prefix_) + ".depth";
      params = &(kparams.depth_params);
    }
    std::cout << i << " abcde\n";
    /*
    params->width = bot_param_get_int_or_fail(bot_param_, (key_prefix_str+".width").c_str());
    params->height = bot_param_get_int_or_fail(bot_param_,(key_prefix_str+".height").c_str());
    params->fx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fx").c_str());
    params->fy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".fy").c_str());
    params->cx = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cx").c_str());
    params->cy = bot_param_get_double_or_fail(bot_param_, (key_prefix_str+".cy").c_str());
    */
  }

  kparams.width = kparams.rgb_params.width;
  kparams.height = kparams.rgb_params.width;
  /*
  kparams.shift_offset = bot_param_get_double_or_fail(
      bot_param_, (key_prefix_+".shift_offset").c_str());
  kparams.projector_depth_baseline = bot_param_get_double_or_fail(
      bot_param_, (key_prefix_+".projector_depth_baseline").c_str());
  */

  Eigen::Matrix3d R;
  R << 0.999999, -0.000796, 0.001256, 0.000739, 0.998970, 0.045368, -0.001291, -0.045367, 0.998970;
  kparams.depth_to_rgb_translation[0] = -0.015756;
  kparams.depth_to_rgb_translation[1] = -0.000923;
  kparams.depth_to_rgb_translation[2] =  0.002316;
  Eigen::Quaterniond Q(R);
  kparams.depth_to_rgb_quaternion[0] = Q.w();
  kparams.depth_to_rgb_quaternion[1] = Q.x();
  kparams.depth_to_rgb_quaternion[2] = Q.y();
  kparams.depth_to_rgb_quaternion[3] = Q.z();

  // We assume rotation is a rotation matrix
  double rotation[9], translation[3];
  /*
  bot_param_get_double_array_or_fail(bot_param_,
                                     (key_prefix_str+".rotation").c_str(),
                                     &rotation[0],
                                     9);
  bot_param_get_double_array_or_fail(bot_param_,
                                     (key_prefix_str+".translation").c_str(),
                                     &translation[0],
                                     3);

  bot_matrix_to_quat(rotation, kparams.depth_to_rgb_quaternion);
  std::copy(translation, translation+3, kparams.depth_to_rgb_translation);
  */
  return std::shared_ptr<fovis::PrimeSenseCalibration>(new fovis::PrimeSenseCalibration(kparams));
}



bool parseBoolean(const std::string &str) {
  if (str.compare("true") == 0) {
    return true;
  } else {
    return false;
  }
}



std::shared_ptr<fovis::StereoCalibration>
KmclConfiguration::load_stereo_calibration() const {
  assert (depth_source_type_==STEREO);


  cv::FileStorage file(config_filename_, cv::FileStorage::READ);
  if (!file.isOpened()) {
    std::cout << "could not read the yaml config file\n";
    exit(-1);
    //return false;
  }

  if (depth_source_type_ != STEREO) {
    return std::shared_ptr<fovis::StereoCalibration>();
  }
  fovis::StereoCalibrationParameters stereo_params;
  fovis::CameraIntrinsicsParameters* params;

  for (int i=0; i < 2; ++i) {
    if (i == 0) {
      params = &(stereo_params.left_parameters);
    } else {
      params = &(stereo_params.right_parameters);
    }

    std::cout << "load stereo" << i << "\n";

    double focal_length_x = static_cast<double>(file["cameras"][i]["focal_length"][0]);
    std::cout << focal_length_x << " focal_length_x\n";

    params->width = static_cast<double>(file["cameras"][i]["image_dimensions"][0]);
    params->height = static_cast<double>(file["cameras"][i]["image_dimensions"][1]);

    params->fx = static_cast<double>(file["cameras"][i]["focal_length"][0]);
    params->fy = static_cast<double>(file["cameras"][i]["focal_length"][1]);
    params->cx = static_cast<double>(file["cameras"][i]["focal_point"][0]);
    params->cy = static_cast<double>(file["cameras"][i]["focal_point"][1]);


    params->k1 = static_cast<double>(file["cameras"][i]["distortion_k1"]);
    params->k2 = static_cast<double>(file["cameras"][i]["distortion_k2"]);
    params->k3 = static_cast<double>(file["cameras"][i]["distortion_k3"]);
    params->p1 = static_cast<double>(file["cameras"][i]["distortion_p1"]);
    params->p2 = static_cast<double>(file["cameras"][i]["distortion_p2"]);
  }

  // We assume rotation is a rotation matrix
  std::string main_key_prefix_str = std::string(key_prefix_);
  double rotation[9], translation[3];

  double quat[4];
  quat[0] = 1;
  quat[1] = 0;
  quat[2] = 0;
  quat[3] = 0;

  translation[0] = -1 * static_cast<double>(file["cameras"][0]["stereo_baseline"]); // actual value should be negative e.g. -0.05
  translation[1] = 0;
  translation[2] = 0;
  /*
  bot_param_get_double_array_or_fail(bot_param_,
                                     (main_key_prefix_str+".rotation").c_str(),
                                     &rotation[0],
                                     9);
  bot_param_get_double_array_or_fail(bot_param_,
                                     (main_key_prefix_str+".translation").c_str(),
                                     &translation[0],
                                     3);

  bot_matrix_to_quat(rotation, stereo_params.right_to_left_rotation);
  */


  std::copy(quat, quat+4, stereo_params.right_to_left_rotation);
  std::copy(translation, translation+3, stereo_params.right_to_left_translation);

  return std::shared_ptr<fovis::StereoCalibration>(new fovis::StereoCalibration(stereo_params));
}

void KmclConfiguration::set_vo_option_int(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  int ival;
}

void KmclConfiguration::set_vo_option_double(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  double dval;
}

void KmclConfiguration::set_vo_option_boolean(fovis::VisualOdometryOptions & vo_opts,
    const std::string & option) const
{
  int bval;
}

fovis::VisualOdometryOptions KmclConfiguration::visual_odometry_options() const {
  fovis::VisualOdometryOptions vo_opts = fovis::VisualOdometry::getDefaultOptions();

  set_vo_option_int(vo_opts, "feature-window-size");
  set_vo_option_int(vo_opts, "max-pyramid-level");
  set_vo_option_int(vo_opts, "min-pyramid-level");
  set_vo_option_int(vo_opts, "target-pixels-per-feature");
  set_vo_option_int(vo_opts, "fast-threshold");
  set_vo_option_double(vo_opts, "fast-threshold-adaptive-gain");
  set_vo_option_boolean(vo_opts, "use-adaptive-threshold");
  set_vo_option_boolean(vo_opts, "use-homography-initialization");
  set_vo_option_int(vo_opts, "ref-frame-change-threshold");

  // OdometryFrame
  set_vo_option_boolean(vo_opts, "use-bucketing");
  set_vo_option_int(vo_opts, "bucket-width");
  set_vo_option_int(vo_opts, "bucket-height");
  set_vo_option_int(vo_opts, "max-keypoints-per-bucket");
  set_vo_option_boolean(vo_opts, "use-image-normalization");

  // MotionEstimator
  set_vo_option_double(vo_opts, "inlier-max-reprojection-error");
  set_vo_option_double(vo_opts, "clique-inlier-threshold");
  set_vo_option_int(vo_opts, "min-features-for-estimate");
  set_vo_option_double(vo_opts, "max-mean-reprojection-error");
  set_vo_option_boolean(vo_opts, "use-subpixel-refinement");
  set_vo_option_int(vo_opts, "feature-search-window");
  set_vo_option_boolean(vo_opts, "update-target-features-with-refined");

  // StereoDepth
  set_vo_option_boolean(vo_opts, "stereo-require-mutual-match");
  set_vo_option_double(vo_opts, "stereo-max-dist-epipolar-line");
  set_vo_option_double(vo_opts, "stereo-max-refinement-displacement");
  set_vo_option_int(vo_opts, "stereo-max-disparity");

  return vo_opts;
}

} // namespace vs
