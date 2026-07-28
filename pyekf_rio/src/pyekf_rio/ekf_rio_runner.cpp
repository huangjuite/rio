#include <pyekf_rio/ekf_rio_runner.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <pyekf_rio/frames.h>

namespace pyekf_rio
{
namespace
{
/// Sign-flip matrix mapping the 22-element error state between NED/FRD and
/// FLU. Each 3-vector block picks up M; the altimeter bias is a scalar.
Eigen::MatrixXd errorStateFrameChange(const rio::EkfRioFilterStateIdx& idx)
{
  Eigen::MatrixXd D = Eigen::MatrixXd::Identity(idx.base_state_length, idx.base_state_length);
  for (unsigned int i : {idx.position, idx.velocity, idx.attitude, idx.bias_acc, idx.bias_gyro, idx.l_b_r, idx.eul_b_r})
    D.block(i, i, 3, 3) = M();
  return D;
}

RioState makeState(const rio::EkfRioFilter& filter, bool initialized)
{
  RioState s;
  s.initialized = initialized;
  if (!initialized)
    return s;

  const rio::NavigationSolution nav = filter.getNavigationSolution();
  const Eigen::Matrix3d C_n_b       = nav.getC_n_b();

  s.t        = filter.getTimestamp().toSec();
  s.position = nedToFlu(nav.getPosition_n_b());
  s.velocity = nedToFlu(nav.v_n_b);

  const Eigen::Quaterniond q_flu(nedToFluRotation(C_n_b));
  s.quaternion = Eigen::Vector4d(q_flu.w(), q_flu.x(), q_flu.y(), q_flu.z());

  const rio::EulerAngles eul = nav.getEuler_n_b();
  s.euler                    = Eigen::Vector3d(eul.roll(), eul.pitch(), eul.yaw());

  s.velocity_body = fluToFrd(C_n_b.transpose() * nav.v_n_b);

  const rio::Offsets bias = filter.getBias();
  s.bias_acc              = fluToFrd(bias.acc);
  s.bias_gyro             = fluToFrd(bias.gyro);
  s.bias_alt              = bias.alt;

  // The clone is always removed before this runs, so the covariance is the
  // 22x22 base state.
  const Eigen::MatrixXd P = filter.getCovarianceMatrix();
  s.covariance_ned        = P;
  const rio::EkfRioFilterStateIdx idx = filter.getErrorIdx();
  if (P.rows() == idx.base_state_length)
  {
    const Eigen::MatrixXd D = errorStateFrameChange(idx);
    s.covariance            = D * P * D.transpose();
  }
  else
  {
    s.covariance = P;
  }

  return s;
}
}  // namespace

sensor_msgs::PointCloud2 makeRadarScan(const Eigen::MatrixXd& pc, const RioConfig& config)
{
  if (pc.cols() != 4 && pc.cols() != 6)
    throw std::invalid_argument("point cloud must be (N, 4) as (x, y, z, doppler) or (N, 6) with (snr_db, noise_db)");

  const double snr_default = (config.default_snr_db > config.min_db) ? config.default_snr_db : config.min_db + 1.0;

  sensor_msgs::PointCloud2 msg;
  msg.points.reserve(pc.rows());

  for (Eigen::Index i = 0; i < pc.rows(); ++i)
  {
    const Eigen::Vector3d p_rfu = radarFluToRfu(pc.row(i).head<3>().transpose());

    sensor_msgs::RadarDetection d;
    d.x             = static_cast<float>(p_rfu.x());
    d.y             = static_cast<float>(p_rfu.y());
    d.z             = static_cast<float>(p_rfu.z());
    d.v_doppler_mps = static_cast<float>(pc(i, 3));
    d.snr_db        = static_cast<float>(pc.cols() == 6 ? pc(i, 4) : snr_default);
    d.noise_db      = static_cast<float>(pc.cols() == 6 ? pc(i, 5) : 0.0);
    msg.points.push_back(d);
  }

  return msg;
}

namespace
{
/// Run reve on an already-built scan and fill the geometric part of the result.
/// `inliers` come back in the caller's radar FLU frame.
RadarResult runEstimator(const reve::RadarEgoVelocityEstimator& estimator,
                         const sensor_msgs::PointCloud2& msg,
                         Eigen::Index n_input)
{
  RadarResult res;
  res.n_input_points = static_cast<int>(n_input);

  rio::Vector3 v_r;
  rio::Matrix3 P_v_r;
  pcl::PointCloud<reve::RadarPointCloudType> inliers;

  // `estimate` is not const upstream; the estimator is otherwise stateless
  // apart from its cached config and RANSAC iteration count.
  res.velocity_valid =
      const_cast<reve::RadarEgoVelocityEstimator&>(estimator).estimate(msg, v_r, P_v_r, inliers);

  if (!res.velocity_valid)
    return res;

  res.v_radar       = v_r;
  res.cov_v_radar   = P_v_r;
  res.sigma_v_radar = Eigen::Vector3d(P_v_r(0, 0), P_v_r(1, 1), P_v_r(2, 2)).array().sqrt();
  res.n_inliers     = static_cast<int>(inliers.size());

  res.inlier_points.resize(inliers.size(), 4);
  for (std::size_t i = 0; i < inliers.size(); ++i)
  {
    const auto& p = inliers.at(i);
    // Back into radar FLU, and undo the doppler sign flip reve applies when
    // it stores inliers (`v_doppler_mps = -v_d`).
    const Eigen::Vector3d p_flu = K().transpose() * Eigen::Vector3d(p.x, p.y, p.z);
    res.inlier_points.row(i) << p_flu.x(), p_flu.y(), p_flu.z(), p.v_doppler_mps;
  }

  return res;
}
}  // namespace

// --- EgoVelocityEstimator --------------------------------------------------

EgoVelocityEstimator::EgoVelocityEstimator(const RioConfig& config) : config_{config}
{
  config_.finalize();
  estimator_.configure(config_);
}

RadarResult EgoVelocityEstimator::estimate(const Eigen::MatrixXd& pc) const
{
  const sensor_msgs::PointCloud2 msg = makeRadarScan(pc, config_);
  RadarResult res                    = runEstimator(estimator_, msg, pc.rows());

  if (res.velocity_valid)
  {
    const Eigen::Quaterniond q_flu(
        config_.q_br_flu_w, config_.q_br_flu_x, config_.q_br_flu_y, config_.q_br_flu_z);
    // v_body(FLU) = C_bflu_rflu * K^T * v_radar(RFU)
    res.v_body = q_flu.normalized().toRotationMatrix() * K().transpose() * res.v_radar;
  }

  return res;
}

// --- EkfRioRunner ----------------------------------------------------------

EkfRioRunner::EkfRioRunner(const RioConfig& config) : config_{config}
{
  config_.finalize();
  estimator_.configure(config_);
  reset();
}

void EkfRioRunner::reset()
{
  initialized_ = false;
  have_last_t_ = false;
  last_imu_t_  = 0.0;
  imu_init_.clear();
  w_window_.clear();
  filter_ = std::make_unique<rio::EkfRioFilter>();
  filter_->configure(config_);
}

bool EkfRioRunner::feedImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double t)
{
  double dt = 0.0;
  if (have_last_t_)
    dt = t - last_imu_t_;
  have_last_t_ = true;
  last_imu_t_  = t;

  // Guard against duplicated or out-of-order samples, which would otherwise
  // produce a non-positive-definite covariance.
  if (dt < 0.0)
    return false;
  dt = std::min(dt, 0.1);

  const rio::ImuDataStamped imu(ros::Time(t), "body", dt, fluToFrd(acc), fluToFrd(gyro));
  last_imu_ = imu;

  // Angular-rate window used to compensate the radar lever arm.
  w_window_.push_back(imu);
  const double window_s = std::max(config_.radar_frame_ms * 1.0e-3, 0.0);
  while (w_window_.size() > 1 && (w_window_.back().time_stamp - w_window_.front().time_stamp).toSec() > window_s)
    w_window_.pop_front();

  if (initialized_)
  {
    filter_->propagate(imu);
    return false;
  }

  imu_init_.push_back(imu);
  const double elapsed = (imu_init_.back().time_stamp - imu_init_.front().time_stamp).toSec();
  if (elapsed > config_.T_init && imu_init_.size() > 1)
  {
    initialized_ = filter_->init(imu_init_, 0.0);
    imu_init_.clear();
    return initialized_;
  }

  return false;
}

Eigen::Vector3d EkfRioRunner::meanAngularRate() const
{
  if (w_window_.empty())
    return last_imu_.w_b_ib;

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& imu : w_window_)
    sum += imu.w_b_ib;
  return sum / static_cast<double>(w_window_.size());
}

RadarResult EkfRioRunner::feedRadar(const Eigen::MatrixXd& pc, double t)
{
  RadarResult res;
  res.n_input_points = static_cast<int>(pc.rows());

  if (!initialized_ || !config_.radar_update)
  {
    res.state = state();
    return res;
  }

  const sensor_msgs::PointCloud2 msg = makeRadarScan(pc, config_);

  filter_->addRadarStateClone(ros::Time(t));

  const Eigen::Vector3d w_mean = config_.use_w ? meanAngularRate() : Eigen::Vector3d::Zero();

  res = runEstimator(estimator_, msg, pc.rows());

  if (res.velocity_valid)
  {
    res.update_applied =
        filter_->updateRadarEgoVelocity(res.v_radar, res.sigma_v_radar, w_mean, config_.outlier_percentil_radar);

    const Eigen::Matrix3d C_b_r = filter_->getCbr();
    // C_b_r maps radar RFU -> body FRD; M brings that back to body FLU.
    res.v_body = fluToFrd(C_b_r * res.v_radar);
  }

  filter_->removeRadarStateClone();

  res.state = state();
  return res;
}

RioState EkfRioRunner::state() const { return makeState(*filter_, initialized_); }
}  // namespace pyekf_rio
