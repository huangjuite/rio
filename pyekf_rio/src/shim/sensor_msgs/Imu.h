// ROS-free stand-in for <sensor_msgs/Imu.h>.
//
// Never constructed by this project, but both `rio_utils/data_types.h` and
// `reve/.../data_types.h` define an inline `ImuDataStamped(ImuConstPtr, Real)`
// constructor and a `toImuMsg()` member, so the type has to exist for those
// headers to compile.

#pragma once

#include <array>
#include <memory>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Header.h>

namespace sensor_msgs
{
struct Imu
{
  std_msgs::Header header;
  geometry_msgs::Quaternion orientation;
  std::array<double, 9> orientation_covariance{};
  geometry_msgs::Vector3 angular_velocity;
  std::array<double, 9> angular_velocity_covariance{};
  geometry_msgs::Vector3 linear_acceleration;
  std::array<double, 9> linear_acceleration_covariance{};
};

typedef std::shared_ptr<Imu> ImuPtr;
typedef std::shared_ptr<const Imu> ImuConstPtr;

struct FluidPressure
{
  std_msgs::Header header;
  double fluid_pressure = 0.0;
  double variance       = 0.0;
};

typedef std::shared_ptr<FluidPressure> FluidPressurePtr;
typedef std::shared_ptr<const FluidPressure> FluidPressureConstPtr;
}  // namespace sensor_msgs
