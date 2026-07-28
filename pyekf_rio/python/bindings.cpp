#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <ros/console.h>

#include "pyekf_rio/config.h"
#include "pyekf_rio/ekf_rio_runner.h"
#include "pyekf_rio/frames.h"

namespace nb = nanobind;
using namespace nb::literals;
using namespace pyekf_rio;

#define RW(name) .def_rw(#name, &RioConfig::name)

NB_MODULE(_core, m)
{
  m.doc() = "ROS-free Python bindings for ekf_rio (Doer & Kohl) and reve";

  nb::enum_<rio_shim::Level>(m, "LogLevel")
      .value("DEBUG", rio_shim::Level::Debug)
      .value("INFO", rio_shim::Level::Info)
      .value("WARN", rio_shim::Level::Warn)
      .value("ERROR", rio_shim::Level::Error)
      .value("FATAL", rio_shim::Level::Fatal)
      .value("NONE", rio_shim::Level::None);

  m.def(
      "set_log_level", [](rio_shim::Level level) { rio_shim::logLevel() = level; }, "level"_a,
      "Set the verbosity of the upstream ROS_*_STREAM log macros.");

  m.def("frame_flu_to_frd", []() { return M(); },
        "FLU <-> NED/FRD rotation, diag(1, -1, -1).");
  m.def("frame_radar_flu_to_rfu", []() { return K(); },
        "Radar FLU -> reve's radar RFU rotation, Rz(+pi/2).");

  nb::class_<RioConfig>(m, "RioConfig")
      .def(nb::init<>())
      .def("finalize", &RioConfig::finalize,
           "Compose q_b_r / l_b_r from the FLU mounting. Idempotent.")
      RW(run_without_radar_trigger)
      RW(altimeter_update) RW(radar_update) RW(sigma_altimeter)
      RW(outlier_percentil_radar) RW(use_w) RW(radar_frame_ms) RW(radar_rate)
      RW(T_init) RW(calib_gyro) RW(g_n)
      RW(p_0_x) RW(p_0_y) RW(p_0_z)
      RW(v_0_x) RW(v_0_y) RW(v_0_z)
      RW(yaw_0_deg)
      RW(b_0_a_x) RW(b_0_a_y) RW(b_0_a_z)
      RW(b_0_w_x_deg) RW(b_0_w_y_deg) RW(b_0_w_z_deg)
      RW(b_0_alt)
      RW(l_b_r_x) RW(l_b_r_y) RW(l_b_r_z)
      RW(q_b_r_w) RW(q_b_r_x) RW(q_b_r_y) RW(q_b_r_z)
      RW(sigma_p) RW(sigma_v) RW(sigma_roll_pitch_deg) RW(sigma_yaw_deg)
      RW(sigma_b_a) RW(sigma_b_w_deg) RW(sigma_b_alt)
      RW(sigma_l_b_r_x) RW(sigma_l_b_r_y) RW(sigma_l_b_r_z)
      RW(sigma_eul_b_r_roll_deg) RW(sigma_eul_b_r_pitch_deg) RW(sigma_eul_b_r_yaw_deg)
      RW(noise_psd_a) RW(noise_psd_w_deg) RW(noise_psd_b_a) RW(noise_psd_b_w_deg)
      RW(noise_psd_b_alt)
      RW(min_dist) RW(max_dist) RW(min_db)
      RW(elevation_thresh_deg) RW(azimuth_thresh_deg)
      RW(filter_min_z) RW(filter_max_z)
      RW(doppler_velocity_correction_factor)
      RW(thresh_zero_velocity) RW(allowed_outlier_percentage)
      RW(sigma_zero_velocity_x) RW(sigma_zero_velocity_y) RW(sigma_zero_velocity_z)
      RW(sigma_offset_radar_x) RW(sigma_offset_radar_y) RW(sigma_offset_radar_z)
      RW(max_sigma_x) RW(max_sigma_y) RW(max_sigma_z) RW(max_r_cond)
      RW(use_cholesky_instead_of_bdcsvd)
      RW(use_ransac) RW(outlier_prob) RW(success_prob) RW(N_ransac_points)
      RW(inlier_thresh)
      RW(use_odr) RW(min_speed_odr) RW(sigma_v_d)
      RW(model_noise_offset_deg) RW(model_noise_scale_deg)
      RW(q_br_flu_w) RW(q_br_flu_x) RW(q_br_flu_y) RW(q_br_flu_z)
      RW(l_br_flu_x) RW(l_br_flu_y) RW(l_br_flu_z)
      RW(use_flu_extrinsics) RW(default_snr_db);

  m.def("load_config", &loadConfig, "path"_a,
        "Load a RioConfig from YAML; unknown keys are ignored.");

  nb::class_<RioState>(m, "RioState")
      .def_ro("t", &RioState::t)
      .def_ro("initialized", &RioState::initialized)
      .def_ro("position", &RioState::position)
      .def_ro("quaternion", &RioState::quaternion)
      .def_ro("euler", &RioState::euler)
      .def_ro("velocity", &RioState::velocity)
      .def_ro("velocity_body", &RioState::velocity_body)
      .def_ro("bias_acc", &RioState::bias_acc)
      .def_ro("bias_gyro", &RioState::bias_gyro)
      .def_ro("bias_alt", &RioState::bias_alt)
      .def_ro("covariance", &RioState::covariance)
      .def_ro("covariance_ned", &RioState::covariance_ned);

  nb::class_<RadarResult>(m, "RadarResult")
      .def_ro("velocity_valid", &RadarResult::velocity_valid)
      .def_ro("update_applied", &RadarResult::update_applied)
      .def_ro("n_input_points", &RadarResult::n_input_points)
      .def_ro("n_inliers", &RadarResult::n_inliers)
      .def_ro("v_radar", &RadarResult::v_radar)
      .def_ro("sigma_v_radar", &RadarResult::sigma_v_radar)
      .def_ro("v_body", &RadarResult::v_body)
      .def_ro("cov_v_radar", &RadarResult::cov_v_radar)
      .def_ro("inlier_points", &RadarResult::inlier_points)
      .def_ro("state", &RadarResult::state);

  nb::class_<EgoVelocityEstimator>(m, "EgoVelocityEstimator")
      .def(nb::init<const RioConfig&>(), "config"_a)
      .def("estimate", &EgoVelocityEstimator::estimate, "pc"_a,
           nb::call_guard<nb::gil_scoped_release>(),
           "Estimate radar ego velocity from an (N, 4) (x, y, z, doppler) or "
           "(N, 6) (..., snr_db, noise_db) array in radar FLU coordinates.")
      .def_prop_ro("config", &EgoVelocityEstimator::config);

  nb::class_<EkfRioRunner>(m, "EkfRioRunner")
      .def(nb::init<const RioConfig&>(), "config"_a)
      .def("reset", &EkfRioRunner::reset)
      .def("feed_imu", &EkfRioRunner::feedImu, "acc"_a, "gyro"_a, "t"_a,
           "Feed one body-FLU IMU sample; returns True on the sample that "
           "completes initialization.")
      .def("feed_radar", &EkfRioRunner::feedRadar, "pc"_a, "t"_a,
           nb::call_guard<nb::gil_scoped_release>(),
           "Feed one radar scan in radar FLU coordinates.")
      .def_prop_ro("initialized", &EkfRioRunner::initialized)
      .def_prop_ro("state", &EkfRioRunner::state)
      .def_prop_ro("config", &EkfRioRunner::config);
}
