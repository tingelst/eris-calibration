// Copyright 2020 Norwegian University of Science and Technology.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <eris/laser_2d_solver.hpp>

namespace eris::hand_eye_calibration::laser2d
{
struct PlanePlus
{
  template <typename T>
  bool operator()(const T* x, const T* delta, T* x_plus_delta) const
  {
    T n_plus_delta[3];
    n_plus_delta[0] = x[0] + delta[0];
    n_plus_delta[1] = x[1] + delta[1];
    n_plus_delta[2] = x[2] + delta[2];

    const T norm_n_plus_delta = sqrt(n_plus_delta[0] * n_plus_delta[0] + n_plus_delta[1] * n_plus_delta[1] + n_plus_delta[2] * n_plus_delta[2]);

    x_plus_delta[0] = n_plus_delta[0] / norm_n_plus_delta;
    x_plus_delta[1] = n_plus_delta[1] / norm_n_plus_delta;
    x_plus_delta[2] = n_plus_delta[2] / norm_n_plus_delta;
    x_plus_delta[3] = x[3] + delta[3];

    return true;
  }
};

auto Solver::AddResidualBlock(const Eigen::Vector4d& quat_i, const Eigen::Vector3d& trs_i, const Eigen::Vector3d& point_i) -> bool
{
  ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 4, 3, 4>(new CostFunctor(quat_i, trs_i, point_i));
  problem_.AddResidualBlock(cost_function, NULL, quat_opt_.data(), trs_opt_.data(), plane_opt_.data());
  return true;
}

auto Solver::Solve() -> std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector4d>
{
  if (!local_parameterization_is_set_)
  {
    problem_.SetParameterization(quat_opt_.data(), new ceres::QuaternionParameterization());
    problem_.SetParameterization(plane_opt_.data(), new ceres::AutoDiffLocalParameterization<PlanePlus, 4, 4>);
    local_parameterization_is_set_ = true;
  }

  options_.linear_solver_type = ceres::DENSE_QR;

  ceres::Solve(options_, &problem_, &summary_);
  return std::make_tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector4d>(std::move(quat_opt_), std::move(trs_opt_), std::move(plane_opt_));
};

auto Solver::Options() -> ceres::Solver::Options
{
  return options_;
}

auto Solver::Summary() -> ceres::Solver::Summary
{
  return summary_;
}

}  // namespace eris::hand_eye_calibration::laser2d
