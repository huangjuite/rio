// Plain-struct replacement for the dynamic_reconfigure-generated EkfRioConfig.
//
// Upstream, `ekf_rio/cfg/EkfRio.cfg` feeds *both* `cfg_ekf_rio.ekf_rio` and
// `cfg_radar_ego_velocity_estimation.radar_ego_velocity_estimator` into one
// ParameterGenerator, and `EkfRioRos::reconfigureCallback` hands the same
// object to `EkfRioFilter::configure()` and
// `RadarEgoVelocityEstimator::configure()`. Both are templated on the config
// container, so this single flat struct serves both.
//
// Defaults are taken from the corresponding `.cfg` files, with one deliberate
// exception noted at `q_b_r_w`.

#pragma once

#include <string>

#include <Eigen/Dense>

namespace pyekf_rio
{
struct RioConfig
{
  // --- filter mode -------------------------------------------------------
  bool run_without_radar_trigger = true;  ///< no trigger topic outside ROS

  // --- KF updates --------------------------------------------------------
  bool altimeter_update  = false;
  bool radar_update      = true;
  double sigma_altimeter = 1.0;

  // --- radar measurement model ------------------------------------------
  double outlier_percentil_radar = 0.05;
  bool use_w                     = true;
  double radar_frame_ms          = 0.0;
  double radar_rate              = 10.0;

  // --- initialization ----------------------------------------------------
  double T_init   = 10.0;
  bool calib_gyro = true;
  double g_n      = 9.81;

  // --- initial nominal states -------------------------------------------
  double p_0_x = 0.0, p_0_y = 0.0, p_0_z = 0.0;
  double v_0_x = 0.0, v_0_y = 0.0, v_0_z = 0.0;
  double yaw_0_deg = 0.0;

  double b_0_a_x = 0.0, b_0_a_y = 0.0, b_0_a_z = 0.0;
  double b_0_w_x_deg = 0.0, b_0_w_y_deg = 0.0, b_0_w_z_deg = 0.0;
  double b_0_alt = 0.0;

  double l_b_r_x = 0.0, l_b_r_y = 0.0, l_b_r_z = 0.0;
  // The reconfigure default for q_b_r_w is 0, which is a degenerate zero
  // quaternion; use the identity instead.
  double q_b_r_w = 1.0, q_b_r_x = 0.0, q_b_r_y = 0.0, q_b_r_z = 0.0;

  // --- initial uncertainty ----------------------------------------------
  double sigma_p               = 0.0;
  double sigma_v               = 0.0;
  double sigma_roll_pitch_deg  = 1.0;
  double sigma_yaw_deg         = 1.0;
  double sigma_b_a             = 0.01;
  double sigma_b_w_deg         = 0.00035;
  double sigma_b_alt           = 0.0;
  double sigma_l_b_r_x = 0.0, sigma_l_b_r_y = 0.0, sigma_l_b_r_z = 0.0;
  double sigma_eul_b_r_roll_deg  = 0.0;
  double sigma_eul_b_r_pitch_deg = 0.0;
  double sigma_eul_b_r_yaw_deg   = 0.0;

  // --- noise PSDs --------------------------------------------------------
  double noise_psd_a       = 0.03;
  double noise_psd_w_deg   = 0.18;
  double noise_psd_b_a     = 0.03;
  double noise_psd_b_w_deg = 0.01;
  double noise_psd_b_alt   = 0.01;

  // --- reve: radar ego velocity estimator --------------------------------
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

  bool use_ransac       = true;
  double outlier_prob   = 0.4;
  double success_prob   = 0.9999;
  int N_ransac_points   = 3;
  double inlier_thresh  = 0.15;

  bool use_odr                  = true;
  double min_speed_odr          = 4.0;
  double sigma_v_d              = 0.125;
  double model_noise_offset_deg = 1.0;
  double model_noise_scale_deg  = 1.0;

  // --- extensions (not upstream) -----------------------------------------
  // Physical radar-to-body mounting expressed in *FLU* conventions, i.e. the
  // rotation taking a radar-FLU vector into body-FLU. `finalize()` composes
  // this with the NED/RFU frame changes to produce upstream's `q_b_r` /
  // `l_b_r`, which are defined radar-RFU -> body-FRD. See frames.h.
  double q_br_flu_w = 1.0, q_br_flu_x = 0.0, q_br_flu_y = 0.0, q_br_flu_z = 0.0;
  double l_br_flu_x = 0.0, l_br_flu_y = 0.0, l_br_flu_z = 0.0;
  /// When true, `finalize()` overwrites q_b_r / l_b_r from the FLU mounting.
  bool use_flu_extrinsics = true;

  /// Synthetic SNR assigned to detections when the caller supplies an (N, 4)
  /// point cloud with no SNR column. Must exceed `min_db` or reve discards
  /// every detection.
  double default_snr_db = 0.0;

  /// Compose `q_b_r` / `l_b_r` from the FLU mounting. Idempotent.
  void finalize();
};

/// Load a RioConfig from a YAML file. Unknown keys are ignored; the two dead
/// keys in upstream's default config (`radar_velocity_correction_factor` and
/// `sigma_v_r`) are accepted as aliases for their real names.
RioConfig loadConfig(const std::string& path);
}  // namespace pyekf_rio
