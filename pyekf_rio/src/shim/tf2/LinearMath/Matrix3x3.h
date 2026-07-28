// ROS-free stand-in for <tf2/LinearMath/Matrix3x3.h>.
//
// `getEulerYPR` replicates tf2's branch structure (including the gimbal-lock
// handling and the two-solution convention) so that
// `NavigationSolution::getEuler_n_b` reports the same angles as upstream.
// This path only feeds log messages, so a discrepancy here is cosmetic --
// unlike `Quaternion::setRPY`, which is on the estimation path.

#pragma once

#include <cmath>

#include <tf2/LinearMath/Quaternion.h>

namespace tf2
{
class Matrix3x3
{
public:
  Matrix3x3() { setIdentity(); }

  explicit Matrix3x3(const Quaternion& q) { setRotation(q); }

  void setIdentity()
  {
    m_el[0][0] = 1.0; m_el[0][1] = 0.0; m_el[0][2] = 0.0;
    m_el[1][0] = 0.0; m_el[1][1] = 1.0; m_el[1][2] = 0.0;
    m_el[2][0] = 0.0; m_el[2][1] = 0.0; m_el[2][2] = 1.0;
  }

  void setRotation(const Quaternion& q)
  {
    const double n = q.length2();
    const double s = (n > 0.0) ? 2.0 / n : 0.0;

    const double xs = q.x() * s,  ys = q.y() * s,  zs = q.z() * s;
    const double wx = q.w() * xs, wy = q.w() * ys, wz = q.w() * zs;
    const double xx = q.x() * xs, xy = q.x() * ys, xz = q.x() * zs;
    const double yy = q.y() * ys, yz = q.y() * zs, zz = q.z() * zs;

    m_el[0][0] = 1.0 - (yy + zz); m_el[0][1] = xy - wz;         m_el[0][2] = xz + wy;
    m_el[1][0] = xy + wz;         m_el[1][1] = 1.0 - (xx + zz); m_el[1][2] = yz - wx;
    m_el[2][0] = xz - wy;         m_el[2][1] = yz + wx;         m_el[2][2] = 1.0 - (xx + yy);
  }

  /// Extract fixed-axis yaw-pitch-roll. `solution_number` selects between the
  /// two valid decompositions, exactly as tf2 does.
  void getEulerYPR(double& yaw, double& pitch, double& roll, unsigned int solution_number = 1) const
  {
    struct Euler
    {
      double yaw;
      double pitch;
      double roll;
    };

    Euler euler_out;
    Euler euler_out2;

    if (std::fabs(m_el[2][0]) >= 1.0)
    {
      euler_out.yaw  = 0.0;
      euler_out2.yaw = 0.0;

      if (m_el[2][0] < 0.0)
      {
        euler_out.pitch  = M_PI / 2.0;
        euler_out2.pitch = M_PI / 2.0;
        euler_out.roll   = std::atan2(m_el[0][1], m_el[0][2]);
        euler_out2.roll  = std::atan2(m_el[0][1], m_el[0][2]);
      }
      else
      {
        euler_out.pitch  = -M_PI / 2.0;
        euler_out2.pitch = -M_PI / 2.0;
        euler_out.roll   = std::atan2(-m_el[0][1], -m_el[0][2]);
        euler_out2.roll  = std::atan2(-m_el[0][1], -m_el[0][2]);
      }
    }
    else
    {
      euler_out.pitch  = -std::asin(m_el[2][0]);
      euler_out2.pitch = M_PI - euler_out.pitch;

      euler_out.roll  = std::atan2(m_el[2][1] / std::cos(euler_out.pitch),
                                   m_el[2][2] / std::cos(euler_out.pitch));
      euler_out2.roll = std::atan2(m_el[2][1] / std::cos(euler_out2.pitch),
                                   m_el[2][2] / std::cos(euler_out2.pitch));

      euler_out.yaw  = std::atan2(m_el[1][0] / std::cos(euler_out.pitch),
                                  m_el[0][0] / std::cos(euler_out.pitch));
      euler_out2.yaw = std::atan2(m_el[1][0] / std::cos(euler_out2.pitch),
                                  m_el[0][0] / std::cos(euler_out2.pitch));
    }

    if (solution_number == 1)
    {
      yaw   = euler_out.yaw;
      pitch = euler_out.pitch;
      roll  = euler_out.roll;
    }
    else
    {
      yaw   = euler_out2.yaw;
      pitch = euler_out2.pitch;
      roll  = euler_out2.roll;
    }
  }

  double m_el[3][3];
};
}  // namespace tf2
