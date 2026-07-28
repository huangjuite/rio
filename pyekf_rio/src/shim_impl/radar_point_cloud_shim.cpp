// Replacement for reve/radar_ego_velocity_estimator/src/radar_point_cloud.cpp.
//
// Upstream that file unpacks a serialized PointCloud2 with pcl_conversions.
// Here the shim `sensor_msgs::PointCloud2` already carries parsed detections,
// so the conversion is a field copy. The declarations in
// `radar_ego_velocity_estimator/radar_point_cloud.h` must be matched exactly
// -- in particular `pclToPcl2msg` takes its cloud *by value*.

#include <cmath>

#include <radar_ego_velocity_estimator/radar_point_cloud.h>

namespace reve
{
bool pcl2msgToPcl(const sensor_msgs::PointCloud2& pcl_msg, pcl::PointCloud<RadarPointCloudType>& scan)
{
  scan.clear();
  scan.reserve(pcl_msg.points.size());

  for (const auto& d : pcl_msg.points)
  {
    RadarPointCloudType p;
    p.x             = d.x;
    p.y             = d.y;
    p.z             = d.z;
    p.snr_db        = d.snr_db;
    p.noise_db      = d.noise_db;
    p.v_doppler_mps = d.v_doppler_mps;
    p.range         = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    scan.push_back(p);
  }

  scan.height   = 1;
  scan.is_dense = true;

  return true;
}

bool pclToPcl2msg(pcl::PointCloud<RadarPointCloudType> scan, sensor_msgs::PointCloud2& pcl_msg)
{
  pcl_msg.points.clear();
  pcl_msg.points.reserve(scan.size());

  for (const auto& p : scan)
  {
    sensor_msgs::RadarDetection d;
    d.x             = p.x;
    d.y             = p.y;
    d.z             = p.z;
    d.snr_db        = p.snr_db;
    d.noise_db      = p.noise_db;
    d.v_doppler_mps = p.v_doppler_mps;
    pcl_msg.points.push_back(d);
  }

  return true;
}
}  // namespace reve
