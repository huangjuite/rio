// Link-time stub for reve::solveODR, compiled instead of reve's odr.cpp when
// PYEKF_RIO_WITH_ODR is OFF (no Fortran compiler available).
//
// `RadarEgoVelocityEstimator::solve3DOdr` only calls this when
// `use_odr` is true, so with `use_odr: false` it is never reached.

#include <radar_ego_velocity_estimator/odr.h>

namespace reve
{
bool solveODR(const Eigen::VectorXd& /*y*/,
              const Eigen::MatrixXd& /*x*/,
              const Eigen::VectorXd& /*sigma_y*/,
              const Eigen::MatrixXd& /*sigma_x*/,
              Eigen::VectorXd& /*beta*/,
              Eigen::VectorXd& /*sigma_beta*/,
              Eigen::MatrixXd& /*covariance_sigma*/)
{
  return false;
}
}  // namespace reve
