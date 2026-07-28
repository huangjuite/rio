// ROS-free stand-in for <std_msgs/Header.h>.

#pragma once

#include <cstdint>
#include <string>

#include <ros/time.h>

namespace std_msgs
{
struct Header
{
  uint32_t seq = 0;
  ros::Time stamp;
  std::string frame_id;
};
}  // namespace std_msgs
