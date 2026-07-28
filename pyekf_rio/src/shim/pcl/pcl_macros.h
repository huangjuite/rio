// ROS-free stand-in for <pcl/pcl_macros.h>.
//
// `POINT_CLOUD_REGISTER_POINT_STRUCT` must be variadic: the upstream call in
// radar_ego_velocity_estimator.cpp is
//     POINT_CLOUD_REGISTER_POINT_STRUCT(RadarPointCloudType, (float, x, x)(...))
// which the preprocessor sees as two arguments.

#pragma once

#include <Eigen/Core>

#define PCL_ADD_UNION_POINT4D                                                            \
  union EIGEN_ALIGN16                                                                    \
  {                                                                                      \
    float data[4];                                                                       \
    struct                                                                               \
    {                                                                                    \
      float x;                                                                           \
      float y;                                                                           \
      float z;                                                                           \
    };                                                                                   \
  }

#define PCL_ADD_POINT4D PCL_ADD_UNION_POINT4D

#define POINT_CLOUD_REGISTER_POINT_STRUCT(...)
#define POINT_CLOUD_REGISTER_POINT_WRAPPER(...)
#define POINT_CLOUD_REGISTER_FIELD_NAME(...)

#ifndef PCL_EXPORTS
#define PCL_EXPORTS
#endif
