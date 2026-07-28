// Stand-in for the dynamic_reconfigure-generated
// <radar_ego_velocity_estimator/RadarEgoVelocityEstimatorConfig.h>.
//
// Unlike EkfRioConfig this cannot be an alias: `RadarEgoVelocityEstimator`
// declares a *member* of this exact type and `configure()` copies into it
// field by field. Namespace and name must match upstream
// (`radar_ego_velocity_estimation`, no `_estimator`).
//
// Defaults from reve's cfg/cfg_radar_ego_velocity_estimation/
// radar_ego_velocity_estimator.py.

#pragma once

namespace radar_ego_velocity_estimation
{
struct RadarEgoVelocityEstimatorConfig
{
  double min_dist                           = 0.25;
  double max_dist                           = 100.0;
  double min_db                             = 5.0;
  double elevation_thresh_deg               = 60.0;
  double azimuth_thresh_deg                 = 60.0;
  double filter_min_z                       = -100.0;
  double filter_max_z                       = 100.0;
  double doppler_velocity_correction_factor = 1.0;

  double thresh_zero_velocity       = 0.05;
  double allowed_outlier_percentage = 0.75;
  double sigma_zero_velocity_x      = 0.01;
  double sigma_zero_velocity_y      = 0.01;
  double sigma_zero_velocity_z      = 0.01;

  double sigma_offset_radar_x = 0.0;
  double sigma_offset_radar_y = 0.0;
  double sigma_offset_radar_z = 0.0;

  double max_sigma_x                  = 0.1;
  double max_sigma_y                  = 0.1;
  double max_sigma_z                  = 0.9;
  double max_r_cond                   = 1.0e3;
  bool use_cholesky_instead_of_bdcsvd = true;

  bool use_ransac      = true;
  double outlier_prob  = 0.4;
  double success_prob  = 0.9999;
  int N_ransac_points  = 3;
  double inlier_thresh = 0.15;

  bool use_odr                  = true;
  double min_speed_odr          = 4.0;
  double sigma_v_d              = 0.125;
  double model_noise_offset_deg = 1.0;
  double model_noise_scale_deg  = 1.0;
};
}  // namespace radar_ego_velocity_estimation
