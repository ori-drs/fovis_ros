/*
 * configuration.hpp
 *
 *  Created on: Jun 5, 2011
 *      Author: Maurice Fallon
 *      Author: Hordur Johannsson
 */

#ifndef KMCL_CONFIGURATION_HPP_
#define KMCL_CONFIGURATION_HPP_

#include <vector>
#include <string>
#include <memory>
#include <fovis/fovis.hpp>
#include <opencv2/core.hpp>

namespace voconfig {

class FovisYAMLConfigurator {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  using StereoCalibPtr = std::shared_ptr<fovis::StereoCalibration>;
  using PrimeSenseCalibPtr = std::shared_ptr<fovis::PrimeSenseCalibration>;

public:
  enum DepthSourceTypeCode {
    UNKNOWN,
    STEREO,
    PRIMESENSE,
    OPENNI
  };

  FovisYAMLConfigurator(const std::string & config_filename);

  virtual ~FovisYAMLConfigurator();

  /**
   * Creates a StereoCalibration block using the provided configuration source and camera.
   *
   * @param config A configuration source to load the configuration from.
   * @param key_prefix names the configuration block that contains
   *                   the calibration information.
   *
   * @return If successful return a shared pointer to the StereoCalibration object
   *         and nullptr otherwise.
   */
  StereoCalibPtr loadStereoCalibration() const;

  /**
   * Creates a PrimeSenseCalibration block using the provided configuration source and camera.
   *
   * @param config A configuration source to load the configuration from.
   * @param key_prefix names the configuration block that contains
   *                   the calibration information.
   *
   * @return If successful return a shared pointer to the PrimeSenseCalibration object
   *         and nullptr otherwise.
   */
  PrimeSenseCalibPtr loadPrimesenseCalibration() const;

  /**
   * @return Parameters for fovis::VisualOdometry
   */

  inline DepthSourceTypeCode getDepthSourceType() const {
    return depth_source_type_;
  }

  inline Eigen::Isometry3d B_t_BC() const {
    return B_t_BC_;
  }


private:
  DepthSourceTypeCode depth_source_type_;

  std::string config_filename_;
  cv::FileStorage file;

  std::string key_prefix_;

  // transformation between base and camera
  // same as rosrun tf tf_echo base camera (in that order)
  Eigen::Isometry3d B_t_BC_;

private:
  Eigen::Vector3d parseVector3d(const cv::FileNode& node);
  bool parseBoolean(const std::string &str);
  void parseBodyToCameraTransform();
};

}

#endif /* CONFIGURATION_HPP_ */
