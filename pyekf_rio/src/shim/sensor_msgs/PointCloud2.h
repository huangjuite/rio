// ROS-free stand-in for <sensor_msgs/PointCloud2.h>.
//
// This is the load-bearing piece of the shim. Upstream, a radar scan reaches
// `reve::RadarEgoVelocityEstimator::estimate` as a serialized PointCloud2 and
// is unpacked by `reve::pcl2msgToPcl`. Here the message simply *carries* the
// six per-detection fields that unpacking would have produced, and our
// `pcl2msgToPcl` (shim_impl/radar_point_cloud_shim.cpp) copies them across.
//
// Note the ordering constraint: `radar_ego_velocity_estimator/radar_point_cloud.h`
// includes this header *before* declaring `reve::RadarPointCloudType`, so the
// payload struct has to be declared here rather than referring to reve's type.

#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include <std_msgs/Header.h>

namespace sensor_msgs
{
/// One radar detection, mirroring the fields of `reve::RadarPointCloudType`.
struct RadarDetection
{
  float x             = 0.0f;
  float y             = 0.0f;
  float z             = 0.0f;
  float snr_db        = 0.0f;
  float noise_db      = 0.0f;
  float v_doppler_mps = 0.0f;
};

/// NOT wire-compatible with the real PointCloud2 -- it holds parsed
/// detections rather than a packed byte blob.
struct PointCloud2
{
  std_msgs::Header header;
  std::vector<RadarDetection> points;
};

typedef std::shared_ptr<PointCloud2> PointCloud2Ptr;
typedef std::shared_ptr<const PointCloud2> PointCloud2ConstPtr;
}  // namespace sensor_msgs
