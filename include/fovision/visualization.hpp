#ifndef __KMCL_visualization_hpp__
#define __KMCL_visualization_hpp__

#include <fovis/fovis.hpp>

#include <bot_lcmgl_client/lcmgl.h>
/*
namespace fovis
{

class VisualOdometry;
class StereoCalibration;
*/

class Visualization {
public:
  Visualization(bot_lcmgl_t* lcmgl, const fovis::StereoCalibration* calib);
  virtual ~Visualization();

  void draw(const fovis::VisualOdometry* odom);
  void draw_pyramid_level_flow(const fovis::VisualOdometry* odom, int level_num);
  void draw_pyramid_level_matches(const fovis::VisualOdometry* odom, int level_num);

private:
  Visualization (const Visualization& other);
  Visualization& operator=(const Visualization& other);

  void colormap(float z, float rgb[3]);

  bot_lcmgl_t* _lcmgl;
  const fovis::StereoCalibration* _calibration;
  float _min_z;
  float _max_z;
};

//}

#endif
