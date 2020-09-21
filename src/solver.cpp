#include <tuple>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Core>

#include <glog/logging.h>
#include <iostream>

namespace eris::hand_eye_calibration
{
class Solver
{
public:
  Solver(const Eigen::Vector4d& q_init, const Eigen::Vector3d t_init) : q_opt_(q_init), t_opt_(t_init){};

  struct CostFunctor
  {
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

  bool AddResidualBlock(const Eigen::Vector4d& qi, const Eigen::Vector3d& ti, const Eigen::Vector3d& pi, const Eigen::Vector4d& qj,
                        const Eigen::Vector3d& tj, const Eigen::Vector3d& pj)
  {
    ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<CostFunctor, 3, 4, 3>(new CostFunctor(qi, ti, pi, qj, tj, pj));
    problem_.AddResidualBlock(cost_function, NULL, q_opt_.data(), t_opt_.data());
    return true;
  }

  auto Solve()
  {
    if (!local_parameterization_is_set_)
    {
      problem_.SetParameterization(q_opt_.data(), new ceres::QuaternionParameterization());
      local_parameterization_is_set_ = true;
    }

    options_.linear_solver_type = ceres::DENSE_QR;
    options_.num_threads = 12;

    ceres::Solve(options_, &problem_, &summary_);
    return std::make_tuple<Eigen::Vector4d, Eigen::Vector3d>(std::move(q_opt_), std::move(t_opt_));
  };

private:
  ceres::Problem problem_;
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;

  bool local_parameterization_is_set_ = false;

  Eigen::Vector4d q_opt_;
  Eigen::Vector3d t_opt_;
};

}  // namespace eris::hand_eye_calibration
