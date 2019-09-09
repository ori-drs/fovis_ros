#include "fovision/visualization.hpp"

#include <iostream>

#include <GL/gl.h>

#include <fovis/fovis.hpp>

using namespace fovis;
/*
namespace fovis
{
*/
static float ORANGE_TO_BLUE_RGB[12][3] = {
   {1.000,   0.167,   0.000},
   {1.000,   0.400,   0.100},
   {1.000,   0.600,   0.200},
   {1.000,   0.800,   0.400},
   {1.000,   0.933,   0.600},
   {1.000,   1.000,   0.800},
   {0.800,   1.000,   1.000},
   {0.600,   0.933,   1.000},
   {0.400,   0.800,   1.000},
   {0.200,   0.600,   1.000},
   {0.100,   0.400,   1.000},
   {0.000,   0.167,   1.000},
};

void
Visualization::colormap(float z, float rgb[3])
{
  // TODO a nonlinear mapping
  float t = std::max(std::min(z, _max_z), _min_z)/(_max_z-_min_z);

  int max_range = sizeof(ORANGE_TO_BLUE_RGB)/sizeof(ORANGE_TO_BLUE_RGB[0])-1;
  int row = static_cast<int>(floor(max_range*t));
  if (row >= max_range) {
    rgb[0] = ORANGE_TO_BLUE_RGB[row][0];
    rgb[1] = ORANGE_TO_BLUE_RGB[row][1];
    rgb[2] = ORANGE_TO_BLUE_RGB[row][2];
    return;
  }
  float w = max_range*t - row;
  rgb[0] = ORANGE_TO_BLUE_RGB[row][0]*w + ORANGE_TO_BLUE_RGB[row+1][0]*(1.-w);
  rgb[1] = ORANGE_TO_BLUE_RGB[row][1]*w + ORANGE_TO_BLUE_RGB[row+1][1]*(1.-w);
  rgb[2] = ORANGE_TO_BLUE_RGB[row][2]*w + ORANGE_TO_BLUE_RGB[row+1][2]*(1.-w);
}

Visualization::Visualization(const StereoCalibration* calib)
  : _calibration(calib)
{
  // take the  Z corresponding to disparity 5 px as 'max Z'
  Eigen::Matrix4d uvdtoxyz = calib->getUvdToXyz();
  Eigen::Vector4d xyzw = uvdtoxyz * Eigen::Vector4d(1, 1, 5, 1);
  xyzw /= xyzw.w();
  _max_z = xyzw.z();

  // take the  Z corresponding to 3/4 disparity img width px as 'min Z'
  xyzw = uvdtoxyz * Eigen::Vector4d(1, 1, (3*calib->getWidth())/4, 1);
  xyzw /= xyzw.w();
  _min_z = xyzw.z();
}

Visualization::~Visualization() {
  //bot_lcmgl_destroy(_lcmgl);
}

void
Visualization::draw(const VisualOdometry* odom)
{
  // all content removed Sept 2019
}

void
Visualization::draw_pyramid_level_flow(const VisualOdometry* odom, int level_num)
{
  // all content removed Sept 2019
}

//}
