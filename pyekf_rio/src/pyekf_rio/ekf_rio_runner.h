// ROS-free driver around `rio::EkfRioFilter` + `reve::RadarEgoVelocityEstimator`.
//
// Replaces `EkfRioRos`, keeping only the estimation logic of its `iterate()`
// loop and dropping the ROS plumbing (node handles, topics, queues, tf, the
// barometer path). The radar handling follows upstream's
// `run_without_radar_trigger` branch exactly:
//
//     addRadarStateClone(stamp)
//       -> mean angular rate over the radar frame
//       -> RadarEgoVelocityEstimator::estimate(...)
//       -> updateRadarEgoVelocity(...)   [only if estimate succeeded]
//     removeRadarStateClone()            [unconditional]
//
// All inputs and outputs at this boundary are FLU (see frames.h); the NED/FRD
// and radar-RFU conversions happen here so callers never see them.

#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <ekf_rio/ekf_rio_filter.h>
#include <radar_ego_velocity_estimator/radar_ego_velocity_estimator.h>

#include <pyekf_rio/config.h>

namespace pyekf_rio
{
/// Filter state snapshot, converted to FLU conventions.
struct RioState
{
  double t          = 0.0;
  bool initialized  = false;

  Eigen::Vector3d position      = Eigen::Vector3d::Zero();  ///< world FLU
  Eigen::Vector4d quaternion    = Eigen::Vector4d(1, 0, 0, 0);  ///< wxyz, world FLU <- body FLU
  Eigen::Vector3d euler         = Eigen::Vector3d::Zero();  ///< roll/pitch/yaw [rad], NED
  Eigen::Vector3d velocity      = Eigen::Vector3d::Zero();  ///< world FLU
  Eigen::Vector3d velocity_body = Eigen::Vector3d::Zero();  ///< body FLU

  Eigen::Vector3d bias_acc  = Eigen::Vector3d::Zero();  ///< body FLU
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();  ///< body FLU
  double bias_alt           = 0.0;

  Eigen::MatrixXd covariance;      ///< 22x22, FLU
  Eigen::MatrixXd covariance_ned;  ///< 22x22, raw filter frame
};

/// Outcome of one radar frame.
struct RadarResult
{
  bool velocity_valid  = false;  ///< reve produced an ego-velocity estimate
  bool update_applied  = false;  ///< the KF update passed the Mahalanobis gate
  int n_input_points   = 0;
  int n_inliers        = 0;

  Eigen::Vector3d v_radar       = Eigen::Vector3d::Zero();  ///< radar RFU
  Eigen::Vector3d sigma_v_radar = Eigen::Vector3d::Zero();  ///< radar RFU
  Eigen::Vector3d v_body        = Eigen::Vector3d::Zero();  ///< body FLU
  Eigen::Matrix3d cov_v_radar   = Eigen::Matrix3d::Zero();

  /// Inlier detections, (M, 4) as (x, y, z, doppler) in the caller's radar FLU
  /// frame.
  Eigen::MatrixXd inlier_points;

  RioState state;
};

/// Thin wrapper exposing reve's estimator on its own, so the ego-velocity
/// stage can be checked against ground truth without the EKF in the loop.
class EgoVelocityEstimator
{
public:
  explicit EgoVelocityEstimator(const RioConfig& config);

  /// `pc` is (N, 4) as (x, y, z, doppler) or (N, 6) with (..., snr_db,
  /// noise_db) appended, in radar FLU.
  RadarResult estimate(const Eigen::MatrixXd& pc) const;

  const RioConfig& config() const { return config_; }

private:
  RioConfig config_;
  reve::RadarEgoVelocityEstimator estimator_;
};

class EkfRioRunner
{
public:
  explicit EkfRioRunner(const RioConfig& config);

  /// Drop all state and start over.
  void reset();

  /// Feed one IMU sample. `acc` [m/s^2] and `gyro` [rad/s] are body FLU.
  /// Returns true on the sample that completes initialization.
  bool feedImu(const Eigen::Vector3d& acc, const Eigen::Vector3d& gyro, double t);

  /// Feed one radar scan; see `EgoVelocityEstimator::estimate` for the layout.
  RadarResult feedRadar(const Eigen::MatrixXd& pc, double t);

  bool initialized() const { return initialized_; }
  RioState state() const;
  const RioConfig& config() const { return config_; }

private:
  Eigen::Vector3d meanAngularRate() const;

  RioConfig config_;
  // Held by pointer: EkfRioFilter is not assignable (EkfRioFilterStateIdx has
  // const members), so `reset()` rebuilds it in place. This also keeps the
  // over-aligned Eigen members off the nanobind-managed allocation.
  std::unique_ptr<rio::EkfRioFilter> filter_;
  reve::RadarEgoVelocityEstimator estimator_;

  bool initialized_ = false;
  bool have_last_t_ = false;
  double last_imu_t_ = 0.0;

  std::vector<rio::ImuDataStamped> imu_init_;
  std::deque<rio::ImuDataStamped> w_window_;
  rio::ImuDataStamped last_imu_;
};

/// Build the shim PointCloud2 reve consumes from an (N, 4) or (N, 6) array of
/// radar-FLU detections. Shared by both classes above.
sensor_msgs::PointCloud2 makeRadarScan(const Eigen::MatrixXd& pc, const RioConfig& config);
}  // namespace pyekf_rio
