// Frame conventions and conversions.
//
// Four frames are in play:
//
//   n      NED world  (x north, y east, z down)          ekf_rio internal
//   b      body FRD   (x forward, y right, z down)       ekf_rio internal
//   w_flu  world FLU  (x forward, y left, z up)          caller / ground truth
//   b_flu  body FLU                                      caller / IMU
//   r      radar RFU  (x right, y forward, z up)         reve internal
//   r_flu  radar FLU  (x forward, y left, z up)          caller's point cloud
//
// `b` is FRD, verified from `math_helper::initFromAcc`
// (roll = -atan2(a_y, -a_z), pitch = asin(a_x / g)) combined with
// `Strapdown` using local_gravity = (0, 0, +9.81) and vdot = C a + g: a level
// static IMU must report a_b = (0, 0, -9.81).
//
// `r` is RFU, verified from reve's `azimuth = atan2(y, x) - pi/2` (azimuth
// zero means x = 0, y > 0, so +y is boresight) plus right-handedness.
//
// Two matrices do all the work, both proper rotations and both involutions
// or exact inverses of each other:
//
//   M = Rx(pi)     = diag(1, -1, -1)     FLU <-> NED / FRD, M = M^T = M^-1
//   K = Rz(+pi/2)  = [[0,-1,0],[1,0,0],[0,0,1]]    radar RFU <- radar FLU

#pragma once

#include <Eigen/Dense>

namespace pyekf_rio
{
/// FLU <-> NED (world) and FLU <-> FRD (body). Self-inverse.
inline const Eigen::Matrix3d& M()
{
  static const Eigen::Matrix3d m = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  return m;
}

/// Radar FLU -> radar RFU: x_rfu = -y_flu, y_rfu = x_flu, z_rfu = z_flu.
inline const Eigen::Matrix3d& K()
{
  static const Eigen::Matrix3d k = [] {
    Eigen::Matrix3d out;
    out << 0.0, -1.0, 0.0,
           1.0,  0.0, 0.0,
           0.0,  0.0, 1.0;
    return out;
  }();
  return k;
}

/// Rotate a body-FLU vector (accelerometer, gyro) into body FRD.
inline Eigen::Vector3d fluToFrd(const Eigen::Vector3d& v) { return M() * v; }

/// Rotate a world-NED vector (position, velocity) into world FLU.
inline Eigen::Vector3d nedToFlu(const Eigen::Vector3d& v) { return M() * v; }

/// Rotate a radar-FLU point into reve's radar RFU frame.
inline Eigen::Vector3d radarFluToRfu(const Eigen::Vector3d& p) { return K() * p; }

/// Convert a NED-world <- FRD-body rotation into FLU-world <- FLU-body.
///
/// Note this is *not* upstream's `getPoseRos()`, which only left-multiplies by
/// Rx(pi) and therefore leaves the child frame FRD. That is fine for RViz but
/// wrong for a FLU-body pose.
inline Eigen::Matrix3d nedToFluRotation(const Eigen::Matrix3d& C_n_b)
{
  return M() * C_n_b * M();
}

/// Compose upstream's `C_b_r` (radar RFU -> body FRD) from the physical
/// mounting expressed in FLU (radar FLU -> body FLU):
///
///     C_b_r = M * C_bflu_rflu * K^T
///
/// For an identity mounting this yields [[0,1,0],[1,0,0],[0,0,-1]], a
/// 180-degree rotation about (1,1,0)/sqrt(2), i.e. the quaternion
/// (w, x, y, z) = (0, sqrt(2)/2, sqrt(2)/2, 0).
inline Eigen::Matrix3d bodyFromRadar(const Eigen::Matrix3d& C_bflu_rflu)
{
  return M() * C_bflu_rflu * K().transpose();
}

/// Compose upstream's `l_b_r` from the FLU mounting translation.
inline Eigen::Vector3d leverArmFromFlu(const Eigen::Vector3d& l_bflu_rflu)
{
  return M() * l_bflu_rflu;
}
}  // namespace pyekf_rio
