#ifndef __KMCL_visualization_hpp__
#define __KMCL_visualization_hpp__

#include <fovis/fovis.hpp>

/*
namespace fovis
{

class VisualOdometry;
class StereoCalibration;
*/

class Visualization {
public:
  Visualization(const fovis::StereoCalibration* calib);
  virtual ~Visualization();

  void draw(const fovis::VisualOdometry* odom);
  void draw_pyramid_level_flow(const fovis::VisualOdometry* odom, int level_num);
  void draw_pyramid_level_matches(const fovis::VisualOdometry* odom, int level_num);

private:
  Visualization (const Visualization& other);
  Visualization& operator=(const Visualization& other);

  void colormap(float z, float rgb[3]);

  const fovis::StereoCalibration* _calibration;
  float _min_z;
  float _max_z;
};

//}

#endif
