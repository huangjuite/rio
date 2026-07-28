// ROS-free stand-in for <angles/angles.h>.
//
// The upstream filter only uses `from_degrees` / `to_degrees`; the rest are
// provided as insurance and match the ROS `angles` package semantics.

#pragma once

#include <cmath>

namespace angles
{
inline double from_degrees(double degrees) { return degrees * M_PI / 180.0; }

inline double to_degrees(double radians) { return radians * 180.0 / M_PI; }

inline double normalize_angle_positive(double angle)
{
  const double two_pi = 2.0 * M_PI;
  return std::fmod(std::fmod(angle, two_pi) + two_pi, two_pi);
}

inline double normalize_angle(double angle)
{
  double a = normalize_angle_positive(angle);
  if (a > M_PI)
    a -= 2.0 * M_PI;
  return a;
}

inline double shortest_angular_distance(double from, double to)
{
  return normalize_angle(to - from);
}
}  // namespace angles
