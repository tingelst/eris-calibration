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

#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

#include <glog/logging.h>
#include <iostream>

namespace eris::hand_eye_calibration
{
class CostFunctor
{
public:
  CostFunctor(const Eigen::Vector4d& qi, const Eigen::Vector3d& ti, const Eigen::Vector3d& pi, const Eigen::Vector4d& qj, const Eigen::Vector3d& tj,
              const Eigen::Vector3d& pj)
    : qi_(qi), ti_(ti), pi_(pi), qj_(qj), tj_(tj), pj_(pj)
  {
  }

  template <typename T>
  auto operator()(const T* const qx, const T* const tx, T* residual) const -> bool
  {
    Eigen::Matrix<T, 4, 1> qi = qi_.cast<T>();
    Eigen::Matrix<T, 3, 1> ti = ti_.cast<T>();
    Eigen::Matrix<T, 3, 1> pi = pi_.cast<T>();
    Eigen::Matrix<T, 4, 1> qj = qj_.cast<T>();
    Eigen::Matrix<T, 3, 1> tj = tj_.cast<T>();
    Eigen::Matrix<T, 3, 1> pj = pj_.cast<T>();

    Eigen::Matrix<T, 3, 1> ppi;
    Eigen::Matrix<T, 3, 1> pppi;
    ceres::QuaternionRotatePoint(qx, pi.data(), ppi.data());
    ppi(0) += tx[0];
    ppi(1) += tx[1];
    ppi(2) += tx[2];
    ceres::QuaternionRotatePoint(qi.data(), ppi.data(), pppi.data());
    pppi(0) += ti[0];
    pppi(1) += ti[1];
    pppi(2) += ti[2];

    Eigen::Matrix<T, 3, 1> ppj;
    Eigen::Matrix<T, 3, 1> pppj;
    ceres::QuaternionRotatePoint(qx, pj.data(), ppj.data());
    ppj(0) += tx[0];
    ppj(1) += tx[1];
    ppj(2) += tx[2];
    ceres::QuaternionRotatePoint(qj.data(), ppj.data(), pppj.data());
    pppj(0) += tj[0];
    pppj(1) += tj[1];
    pppj(2) += tj[2];

    residual[0] = pppj(0) - pppi(0);
    residual[1] = pppj(1) - pppi(1);
    residual[2] = pppj(2) - pppi(2);
    return true;
  }

private:
  const Eigen::Vector4d qi_;
  const Eigen::Vector3d ti_;
  const Eigen::Vector3d pi_;
  const Eigen::Vector4d qj_;
  const Eigen::Vector3d tj_;
  const Eigen::Vector3d pj_;
};

class Solver
{
public:
  Solver(const Eigen::Vector4d& q_init, const Eigen::Vector3d t_init) : q_opt_(q_init), t_opt_(t_init)
  {
  }

  auto AddResidualBlock(const Eigen::Vector4d&, const Eigen::Vector3d&, const Eigen::Vector3d&, const Eigen::Vector4d&, const Eigen::Vector3d&,
                        const Eigen::Vector3d&) -> bool;

  auto Solve() -> std::tuple<Eigen::Vector4d, Eigen::Vector3d>;

  auto Summary() -> ceres::Solver::Summary;

  auto Options() -> ceres::Solver::Options;

private:
  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;

  bool local_parameterization_is_set_ = false;

  Eigen::Vector4d q_opt_;
  Eigen::Vector3d t_opt_;
};
}  // namespace eris::hand_eye_calibration