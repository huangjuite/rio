// ROS-free stand-in for <tf2/LinearMath/Quaternion.h>.
//
// `setRPY` reproduces the bullet/tf2 formula exactly, because
// `NavigationSolution::setEuler_n_b` is on the critical path of
// `EkfRioFilter::init` -- a sign error here shows up as a bad initial
// attitude and quadratic position drift.
//
// Sanity check: setRPY(pi, 0, 0) -> (x, y, z, w) = (1, 0, 0, 0), whose
// rotation matrix is diag(1, -1, -1), matching `getPoseRos()`'s intent.

#pragma once

#include <cmath>

namespace tf2
{
class Quaternion
{
public:
  Quaternion() : x_{0.0}, y_{0.0}, z_{0.0}, w_{1.0} {}

  Quaternion(double x, double y, double z, double w) : x_{x}, y_{y}, z_{z}, w_{w} {}

  /// Set from fixed-axis roll (X), pitch (Y), yaw (Z) -- tf2 convention.
  void setRPY(double roll, double pitch, double yaw)
  {
    const double half_roll  = 0.5 * roll;
    const double half_pitch = 0.5 * pitch;
    const double half_yaw   = 0.5 * yaw;

    const double sr = std::sin(half_roll);
    const double cr = std::cos(half_roll);
    const double sp = std::sin(half_pitch);
    const double cp = std::cos(half_pitch);
    const double sy = std::sin(half_yaw);
    const double cy = std::cos(half_yaw);

    x_ = sr * cp * cy - cr * sp * sy;
    y_ = cr * sp * cy + sr * cp * sy;
    z_ = cr * cp * sy - sr * sp * cy;
    w_ = cr * cp * cy + sr * sp * sy;
  }

  void setValue(double x, double y, double z, double w)
  {
    x_ = x;
    y_ = y;
    z_ = z;
    w_ = w;
  }

  double x() const { return x_; }
  double y() const { return y_; }
  double z() const { return z_; }
  double w() const { return w_; }

  double length2() const { return x_ * x_ + y_ * y_ + z_ * z_ + w_ * w_; }
  double length() const { return std::sqrt(length2()); }

  void normalize()
  {
    const double n = length();
    if (n > 0.0)
    {
      x_ /= n;
      y_ /= n;
      z_ /= n;
      w_ /= n;
    }
  }

private:
  double x_;
  double y_;
  double z_;
  double w_;
};
}  // namespace tf2
