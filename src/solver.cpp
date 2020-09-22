#include <tuple>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

#include <glog/logging.h>
#include <iostream>

#include <eris/solver.hpp>

namespace eris::hand_eye_calibration
{
auto Solver::AddResidualBlock(const Eigen::Vector4d& qi, const Eigen::Vector3d& ti, const Eigen::Vector3d& pi, const Eigen::Vector4d& qj,
                              const Eigen::Vector3d& tj, const Eigen::Vector3d& pj) -> bool
{
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 4, 3>(new CostFunctor(qi, ti, pi, qj, tj, pj));
  problem_.AddResidualBlock(cost_function, NULL, q_opt_.data(), t_opt_.data());
  return true;
}

auto Solver::Solve() -> std::tuple<Eigen::Vector4d, Eigen::Vector3d>
{
  if (!local_parameterization_is_set_)
  {
    problem_.SetParameterization(q_opt_.data(), new ceres::QuaternionParameterization());
    local_parameterization_is_set_ = true;
  }

  options_.linear_solver_type = ceres::DENSE_QR;
  // options_.num_threads = 12;

  ceres::Solve(options_, &problem_, &summary_);
  return std::make_tuple<Eigen::Vector4d, Eigen::Vector3d>(std::move(q_opt_), std::move(t_opt_));
};

auto Solver::Options() -> ceres::Solver::Options
{
  return options_;
}

auto Solver::Summary() -> ceres::Solver::Summary
{
  return summary_;
}

}  // namespace eris::hand_eye_calibration
