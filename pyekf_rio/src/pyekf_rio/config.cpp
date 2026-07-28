#include <pyekf_rio/config.h>

#include <yaml-cpp/yaml.h>

#include <pyekf_rio/frames.h>

namespace pyekf_rio
{
namespace
{
/// Read `key` into `out` if present. Missing keys leave the default in place.
template <typename T>
void get(const YAML::Node& node, const char* key, T& out)
{
  if (node[key])
    out = node[key].as<T>();
}
}  // namespace

void RioConfig::finalize()
{
  if (!use_flu_extrinsics)
    return;

  const Eigen::Quaterniond q_flu(q_br_flu_w, q_br_flu_x, q_br_flu_y, q_br_flu_z);
  const Eigen::Matrix3d C_b_r = bodyFromRadar(q_flu.normalized().toRotationMatrix());
  const Eigen::Quaterniond q_b_r(C_b_r);

  q_b_r_w = q_b_r.w();
  q_b_r_x = q_b_r.x();
  q_b_r_y = q_b_r.y();
  q_b_r_z = q_b_r.z();

  const Eigen::Vector3d l_b_r = leverArmFromFlu(Eigen::Vector3d(l_br_flu_x, l_br_flu_y, l_br_flu_z));
  l_b_r_x = l_b_r.x();
  l_b_r_y = l_b_r.y();
  l_b_r_z = l_b_r.z();
}

RioConfig loadConfig(const std::string& path)
{
  const YAML::Node n = YAML::LoadFile(path);
  RioConfig c;

  get(n, "run_without_radar_trigger", c.run_without_radar_trigger);

  get(n, "altimeter_update", c.altimeter_update);
  get(n, "radar_update", c.radar_update);
  get(n, "sigma_altimeter", c.sigma_altimeter);

  get(n, "outlier_percentil_radar", c.outlier_percentil_radar);
  get(n, "use_w", c.use_w);
  get(n, "radar_frame_ms", c.radar_frame_ms);
  get(n, "radar_rate", c.radar_rate);

  get(n, "T_init", c.T_init);
  get(n, "calib_gyro", c.calib_gyro);
  get(n, "g_n", c.g_n);

  get(n, "p_0_x", c.p_0_x); get(n, "p_0_y", c.p_0_y); get(n, "p_0_z", c.p_0_z);
  get(n, "v_0_x", c.v_0_x); get(n, "v_0_y", c.v_0_y); get(n, "v_0_z", c.v_0_z);
  get(n, "yaw_0_deg", c.yaw_0_deg);

  get(n, "b_0_a_x", c.b_0_a_x); get(n, "b_0_a_y", c.b_0_a_y); get(n, "b_0_a_z", c.b_0_a_z);
  get(n, "b_0_w_x_deg", c.b_0_w_x_deg);
  get(n, "b_0_w_y_deg", c.b_0_w_y_deg);
  get(n, "b_0_w_z_deg", c.b_0_w_z_deg);
  get(n, "b_0_alt", c.b_0_alt);

  get(n, "l_b_r_x", c.l_b_r_x); get(n, "l_b_r_y", c.l_b_r_y); get(n, "l_b_r_z", c.l_b_r_z);
  get(n, "q_b_r_w", c.q_b_r_w); get(n, "q_b_r_x", c.q_b_r_x);
  get(n, "q_b_r_y", c.q_b_r_y); get(n, "q_b_r_z", c.q_b_r_z);

  get(n, "sigma_p", c.sigma_p);
  get(n, "sigma_v", c.sigma_v);
  get(n, "sigma_roll_pitch_deg", c.sigma_roll_pitch_deg);
  get(n, "sigma_yaw_deg", c.sigma_yaw_deg);
  get(n, "sigma_b_a", c.sigma_b_a);
  get(n, "sigma_b_w_deg", c.sigma_b_w_deg);
  get(n, "sigma_b_alt", c.sigma_b_alt);
  get(n, "sigma_l_b_r_x", c.sigma_l_b_r_x);
  get(n, "sigma_l_b_r_y", c.sigma_l_b_r_y);
  get(n, "sigma_l_b_r_z", c.sigma_l_b_r_z);
  get(n, "sigma_eul_b_r_roll_deg", c.sigma_eul_b_r_roll_deg);
  get(n, "sigma_eul_b_r_pitch_deg", c.sigma_eul_b_r_pitch_deg);
  get(n, "sigma_eul_b_r_yaw_deg", c.sigma_eul_b_r_yaw_deg);

  get(n, "noise_psd_a", c.noise_psd_a);
  get(n, "noise_psd_w_deg", c.noise_psd_w_deg);
  get(n, "noise_psd_b_a", c.noise_psd_b_a);
  get(n, "noise_psd_b_w_deg", c.noise_psd_b_w_deg);
  get(n, "noise_psd_b_alt", c.noise_psd_b_alt);

  get(n, "min_dist", c.min_dist);
  get(n, "max_dist", c.max_dist);
  get(n, "min_db", c.min_db);
  get(n, "elevation_thresh_deg", c.elevation_thresh_deg);
  get(n, "azimuth_thresh_deg", c.azimuth_thresh_deg);
  get(n, "filter_min_z", c.filter_min_z);
  get(n, "filter_max_z", c.filter_max_z);
  get(n, "doppler_velocity_correction_factor", c.doppler_velocity_correction_factor);
  // Dead key in upstream's ekf_rio_default.yaml; accept it as an alias.
  get(n, "radar_velocity_correction_factor", c.doppler_velocity_correction_factor);

  get(n, "thresh_zero_velocity", c.thresh_zero_velocity);
  get(n, "allowed_outlier_percentage", c.allowed_outlier_percentage);
  get(n, "sigma_zero_velocity_x", c.sigma_zero_velocity_x);
  get(n, "sigma_zero_velocity_y", c.sigma_zero_velocity_y);
  get(n, "sigma_zero_velocity_z", c.sigma_zero_velocity_z);

  get(n, "sigma_offset_radar_x", c.sigma_offset_radar_x);
  get(n, "sigma_offset_radar_y", c.sigma_offset_radar_y);
  get(n, "sigma_offset_radar_z", c.sigma_offset_radar_z);

  get(n, "max_sigma_x", c.max_sigma_x);
  get(n, "max_sigma_y", c.max_sigma_y);
  get(n, "max_sigma_z", c.max_sigma_z);
  get(n, "max_r_cond", c.max_r_cond);
  get(n, "use_cholesky_instead_of_bdcsvd", c.use_cholesky_instead_of_bdcsvd);

  get(n, "use_ransac", c.use_ransac);
  get(n, "outlier_prob", c.outlier_prob);
  get(n, "success_prob", c.success_prob);
  get(n, "N_ransac_points", c.N_ransac_points);
  get(n, "inlier_thresh", c.inlier_thresh);

  get(n, "use_odr", c.use_odr);
  get(n, "min_speed_odr", c.min_speed_odr);
  get(n, "sigma_v_d", c.sigma_v_d);
  // Dead key in upstream's ekf_rio_default.yaml; accept it as an alias.
  get(n, "sigma_v_r", c.sigma_v_d);
  get(n, "model_noise_offset_deg", c.model_noise_offset_deg);
  get(n, "model_noise_scale_deg", c.model_noise_scale_deg);

  get(n, "q_br_flu_w", c.q_br_flu_w); get(n, "q_br_flu_x", c.q_br_flu_x);
  get(n, "q_br_flu_y", c.q_br_flu_y); get(n, "q_br_flu_z", c.q_br_flu_z);
  get(n, "l_br_flu_x", c.l_br_flu_x); get(n, "l_br_flu_y", c.l_br_flu_y);
  get(n, "l_br_flu_z", c.l_br_flu_z);
  get(n, "use_flu_extrinsics", c.use_flu_extrinsics);
  get(n, "default_snr_db", c.default_snr_db);

  c.finalize();
  return c;
}
}  // namespace pyekf_rio
