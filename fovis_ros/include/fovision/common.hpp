#pragma once

inline Eigen::Quaterniond euler_to_quat(double roll, double pitch, double yaw) {

  // This conversion function introduces a NaN in Eigen Rotations when:
  // roll == pi , pitch,yaw =0    ... or other combinations.
  // cos(pi) ~=0 but not exactly 0
  if ( ((roll==M_PI) && (pitch ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,1,0,0);
  }else if( ((pitch==M_PI) && (roll ==0)) && (yaw ==0)){
    return  Eigen::Quaterniond(0,0,1,0);
  }else if( ((yaw==M_PI) && (roll ==0)) && (pitch ==0)){
    return  Eigen::Quaterniond(0,0,0,1);
  }

  double sy = sin(yaw*0.5);
  double cy = cos(yaw*0.5);
  double sp = sin(pitch*0.5);
  double cp = cos(pitch*0.5);
  double sr = sin(roll*0.5);
  double cr = cos(roll*0.5);
  double w = cr*cp*cy + sr*sp*sy;
  double x = sr*cp*cy - cr*sp*sy;
  double y = cr*sp*cy + sr*cp*sy;
  double z = cr*cp*sy - sr*sp*cy;
  return Eigen::Quaterniond(w,x,y,z);
}

inline void quat_to_euler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2.0*(q0*q1+q2*q3), 1.0-2.0*(q1*q1+q2*q2));
  pitch = asin(2.0*(q0*q2-q3*q1));
  yaw = atan2(2.0*(q0*q3+q1*q2), 1.0-2.0*(q2*q2+q3*q3));
}


// was bot_quat_rotate_to
inline void quat_rotate_to (const double rot[4], const double v[3], double r[3])
{
    double ab  =  rot[0]*rot[1], ac = rot[0]*rot[2], ad  =  rot[0]*rot[3];
    double nbb = -rot[1]*rot[1], bc = rot[1]*rot[2], bd  =  rot[1]*rot[3];
    double ncc = -rot[2]*rot[2], cd = rot[2]*rot[3], ndd = -rot[3]*rot[3];

    r[0] = 2*((ncc + ndd)*v[0] + (bc - ad)*v[1] + (ac + bd)*v[2]) + v[0];
    r[1] = 2*((ad + bc)*v[0] + (nbb + ndd)*v[1] + (cd - ab)*v[2]) + v[1];
    r[2] = 2*((bd - ac)*v[0] + (ab + cd)*v[1] + (nbb + ncc)*v[2]) + v[2];
}

