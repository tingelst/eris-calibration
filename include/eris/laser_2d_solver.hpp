// Copyright 2021 Norwegian University of Science and Technology.
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

#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

#include <glog/logging.h>
#include <iostream>

namespace eris::hand_eye_calibration::laser2d
{
class CostFunctor
{
public:
  CostFunctor(const Eigen::Vector4d& quat_i, 
    const Eigen::Vector3d& trs_i, const Eigen::Vector3d& point_i )
    : quat_i_(quat_i), trs_i_(trs_i), point_i_(point_i)
  {
  }

  template <typename T>
  auto operator()(const T* const quat_x, const T* const trs_x, const T* plane_x, T* residual) const -> bool
  {
    Eigen::Matrix<T, 4, 1> quat_i = quat_i_.cast<T>();
    Eigen::Matrix<T, 3, 1> trs_i = trs_i_.cast<T>();
    Eigen::Matrix<T, 3, 1> point_i = point_i_.cast<T>();

    Eigen::Matrix<T, 3, 1> ppoint_i;
    ceres::QuaternionRotatePoint(quat_x, point_i.data(), ppoint_i.data());
    ppoint_i(0) += trs_x[0];
    ppoint_i(1) += trs_x[1];
    ppoint_i(2) += trs_x[2];

    Eigen::Matrix<T, 3, 1> pppoint_i;
    ceres::QuaternionRotatePoint(quat_i.data(), ppoint_i.data(), pppoint_i.data());
    pppoint_i(0) += trs_i[0];
    pppoint_i(1) += trs_i[1];
    pppoint_i(2) += trs_i[2];

    T a = plane_x[0];
    T b = plane_x[1];
    T c = plane_x[2];
    T d = plane_x[3];

    T k = (a * pppoint_i(0) + b * pppoint_i(1) + c * pppoint_i(2) - d) / (a * a + b * b + c * c);

    residual[0] = k * a;
    residual[1] = k * b;
    residual[2] = k * c;

    return true;
  }

private:
  const Eigen::Vector4d quat_i_;
  const Eigen::Vector3d trs_i_;
  const Eigen::Vector3d point_i_;
};

class Solver
{
public:
  Solver(const Eigen::Vector4d& quat_init, const Eigen::Vector3d trs_init, const Eigen::Vector4d plane_init) : 
   quat_opt_(quat_init), trs_opt_(trs_init), plane_opt_(plane_init)
  {
  }

  auto AddResidualBlock(const Eigen::Vector4d&, const Eigen::Vector3d&, const Eigen::Vector3d&) -> bool;

  auto Solve() -> std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector4d>;

  auto Summary() -> ceres::Solver::Summary;

  auto Options() -> ceres::Solver::Options;

private:
  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;

  bool local_parameterization_is_set_ = false;

  Eigen::Vector4d quat_opt_;
  Eigen::Vector3d trs_opt_;
  Eigen::Vector4d plane_opt_;
};
}  // namespace eris::hand_eye_calibration::laser2d
