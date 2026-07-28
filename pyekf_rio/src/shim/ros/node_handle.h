// ROS-free stand-in for <ros/node_handle.h>.
//
// Reached only because `radar_ego_velocity_estimator/ros_helper.h` is pulled
// in by the estimator header. `getRosParameter` is a template that is never
// instantiated, so an inert NodeHandle is enough.

#pragma once

#include <string>

#include <ros/console.h>
#include <ros/time.h>

namespace ros
{
class NodeHandle
{
public:
  NodeHandle() = default;
  explicit NodeHandle(const std::string&) {}

  template <typename T>
  bool getParam(const std::string&, T&) const
  {
    return false;
  }

  template <typename T>
  void setParam(const std::string&, const T&) const
  {
  }

  template <typename T>
  bool param(const std::string&, T& value, const T& fallback) const
  {
    value = fallback;
    return false;
  }
};
}  // namespace ros
