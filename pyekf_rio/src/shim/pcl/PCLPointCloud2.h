// ROS-free stand-in for <pcl/PCLPointCloud2.h>.
//
// Nothing in the compiled set touches `pcl::PCLPointCloud2` -- the serialized
// form is replaced wholesale by the shim `sensor_msgs::PointCloud2`.

#pragma once

#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
