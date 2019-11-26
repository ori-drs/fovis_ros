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


#include <eigen_utils/eigen_utils.hpp>

using StereoCalibPtr = voconfig::FovisYAMLConfigurator::StereoCalibPtr;
using PrimeSenseCalibPtr = voconfig::FovisYAMLConfigurator::PrimeSenseCalibPtr;

namespace voconfig {

FovisYAMLConfigurator::FovisYAMLConfigurator(const std::string & config_filename) :
  depth_source_type_(UNKNOWN),
  config_filename_(config_filename)
{
  file.open(config_filename_, cv::FileStorage::READ);

  if (!file.isOpened()) {
    std::cout << "could not read the yaml config file\n";
    exit(-1);
  }

  std::string type_str = "stereo";
  try {
    type_str = static_cast<std::string>(file["cameras"][0]["depth_source_type"]);
  } catch(cv::Exception e){
    // silently set the value to stereo if the vallue cannot be read
  }

  if(type_str.compare("stereo") == 0){
    depth_source_type_ = STEREO;
  } else if(type_str.compare("primesense") == 0){
    depth_source_type_ = PRIMESENSE;
  } else if (type_str.compare("openni") == 0) {
    depth_source_type_ = OPENNI;
  } else {
    std::cerr << "[ FovisYAMLConfigurator ] WARNING: unknown depth_source_type."
              << " Setting to STEREO as default." << std::endl;
    depth_source_type_ = STEREO;
  }
  parseBodyToCameraTransform();
}

FovisYAMLConfigurator::~FovisYAMLConfigurator(){
  file.release();
}

void FovisYAMLConfigurator::parseBodyToCameraTransform() {
  Eigen::Vector3d B_r_BC = parseVector3d(file["cameras"][0]["B_r_BC"]);

  Eigen::Quaterniond B_q_BC;
  // Eigen:Quaternion is WXYZ | yaml is XYZW
  B_q_BC.w() = static_cast<double>(file["cameras"][0]["B_q_BC"][3]);
  B_q_BC.x() = static_cast<double>(file["cameras"][0]["B_q_BC"][0]);
  B_q_BC.y() = static_cast<double>(file["cameras"][0]["B_q_BC"][1]);
  B_q_BC.z() = static_cast<double>(file["cameras"][0]["B_q_BC"][2]);

  std::cout << "Received Body to Camera rotation [RPY, deg]: "
            << eigen_utils::getEulerAnglesDeg(B_q_BC).transpose() << std::endl;

  // normalize the quaternion we read from config file to be sure it's valid
  B_q_BC.normalize();

  B_t_BC_ = Eigen::Isometry3d::Identity();
  B_t_BC_.translation() = B_r_BC;
  B_t_BC_.rotate(B_q_BC);

  std::cout << B_r_BC << " [B_r_BC]\n";
  std::cout << B_q_BC.x() << " " << B_q_BC.y() << " " << B_q_BC.z() << " " << B_q_BC.w() << " B_q_BC [xyzw]\n";

  // this should be the same as B_q_BC
  Eigen::Quaterniond B_q_BC2 = Eigen::Quaterniond(B_t_BC_.rotation());
  std::cout << B_q_BC2.x() << " " << B_q_BC2.y() << " " << B_q_BC2.z() << " " << B_q_BC2.w() << " B_q_BC [xyzw]\n";
}

StereoCalibPtr FovisYAMLConfigurator::loadStereoCalibration() const {
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

    std::cout << "Loading stereo configuration" << i << "\n";

    double focal_length_x = static_cast<double>(file["cameras"][i]["focal_length"][0]);
    std::cout << "Focal_length_x: " << focal_length_x << std::endl;

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
  double translation[3];

  // assuming same alignment between the two cameras
  double quat[4];
  quat[0] = 1;
  quat[1] = 0;
  quat[2] = 0;
  quat[3] = 0;

  double stereo_baseline = static_cast<double>(file["cameras"][0]["stereo_baseline"]);
  std::cout << "Stereo baseline: " << stereo_baseline << std::endl;

  // actual value should be negative e.g. -0.05
  translation[0] = -stereo_baseline;
  translation[1] = 0;
  translation[2] = 0;

  std::copy(quat, quat+4, stereo_params.right_to_left_rotation);
  std::copy(translation, translation+3, stereo_params.right_to_left_translation);

  return std::shared_ptr<fovis::StereoCalibration>(new fovis::StereoCalibration(stereo_params));
}

PrimeSenseCalibPtr FovisYAMLConfigurator::loadPrimesenseCalibration() const {
  assert (depth_source_type_== PRIMESENSE || depth_source_type_ == OPENNI);
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
    // TODO: set params->width, height, fx, fy, cx, cy
  }

  kparams.width = kparams.rgb_params.width;
  kparams.height = kparams.rgb_params.width;

  // TODO set kparams.shift_offset, kparams.projector_depth_baseline

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

  // TODO fill kparams.depth_to_rgb_quaternion and kparams.depth_to_rgb_translation
  return PrimeSenseCalibPtr(new fovis::PrimeSenseCalibration(kparams));
}




/// \brief Parse the contents of a 3 element vector.
Eigen::Vector3d FovisYAMLConfigurator::parseVector3d(const cv::FileNode& node) {
  if (node.size()!=3) {
    //std::cerr << "Using defalt value for vector.." << std::endl;
    return Eigen::Vector3d{0,0,0};
  }
  return Eigen::Vector3d{static_cast<double>(node[0]),
                         static_cast<double>(node[1]),
                         static_cast<double>(node[2])};
}

bool FovisYAMLConfigurator::parseBoolean(const std::string &str) {
  if (str.compare("true") == 0) {
    return true;
  } else {
    return false;
  }
}
} // namespace vs
