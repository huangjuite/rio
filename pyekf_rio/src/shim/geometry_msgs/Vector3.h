// ROS-free stand-in for the handful of geometry_msgs types reached by the
// upstream headers.

#pragma once

namespace geometry_msgs
{
struct Vector3
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Point
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct Quaternion
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 1.0;
};
}  // namespace geometry_msgs
