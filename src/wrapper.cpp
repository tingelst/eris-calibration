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

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <eris/solver.hpp>
#include <eris/laser_2d_solver.hpp>

namespace py = pybind11;

auto SummaryToDict(const ceres::Solver::Summary& summary) -> py::dict
{
  py::dict summary_dict;
  summary_dict[py::str("linear_solver_type_used")] = py::str(ceres::LinearSolverTypeToString(summary.linear_solver_type_used));

  summary_dict[py::str("initial_cost")] = py::float_(summary.initial_cost);
  summary_dict[py::str("final_cost")] = py::float_(summary.final_cost);
  summary_dict[py::str("preprocessor_time_in_seconds")] = py::float_(summary.preprocessor_time_in_seconds);
  summary_dict[py::str("postprocessor_time_in_seconds")] = py::float_(summary.postprocessor_time_in_seconds);
  summary_dict[py::str("total_time_in_seconds")] = py::float_(summary.total_time_in_seconds);
  summary_dict[py::str("minimizer_time_in_seconds")] = py::float_(summary.minimizer_time_in_seconds);
  summary_dict[py::str("linear_solver_time_in_seconds")] = py::float_(summary.linear_solver_time_in_seconds);
  summary_dict[py::str("residual_evaluation_time_in_seconds")] = py::float_(summary.residual_evaluation_time_in_seconds);
  summary_dict[py::str("jacobian_evaluation_time_in_seconds")] = py::float_(summary.jacobian_evaluation_time_in_seconds);
  summary_dict[py::str("num_parameter_blocks")] = py::int_(summary.num_parameter_blocks);
  summary_dict[py::str("num_parameters")] = py::int_(summary.num_parameters);
  summary_dict[py::str("num_residuals")] = py::int_(summary.num_residuals);
  summary_dict[py::str("num_residual_blocks")] = py::int_(summary.num_residual_blocks);
  summary_dict[py::str("num_effective_parameters")] = py::int_(summary.num_effective_parameters);
  summary_dict[py::str("num_residual_blocks")] = py::int_(summary.num_residual_blocks);
  summary_dict[py::str("num_residuals")] = py::int_(summary.num_residuals);
  summary_dict[py::str("trust_region_strategy_type")] = py::str(ceres::TrustRegionStrategyTypeToString(summary.trust_region_strategy_type));
  summary_dict[py::str("minimizer_type")] = py::str(ceres::MinimizerTypeToString(summary.minimizer_type));

  summary_dict[py::str("message")] = py::str(summary.message.c_str());
  summary_dict[py::str("num_threads_given")] = py::int_(summary.num_threads_given);
  summary_dict[py::str("num_threads_used")] = py::int_(summary.num_threads_used);

  py::list iterations;
  auto its = summary.iterations;
  for (int i = 0; i < its.size(); ++i)
  {
    py::dict iteration;
    iteration[py::str("iteration")] = py::int_(its[i].iteration);
    iteration[py::str("cost")] = py::float_(its[i].cost);
    iteration[py::str("cost_change")] = py::float_(its[i].cost_change);
    iteration[py::str("gradient_max_norm")] = py::float_(its[i].gradient_max_norm);
    iteration[py::str("step_norm")] = py::float_(its[i].step_norm);
    iteration[py::str("relative_decrease")] = py::float_(its[i].relative_decrease);
    iteration[py::str("trust_region_radius")] = py::float_(its[i].trust_region_radius);
    iteration[py::str("eta")] = py::float_(its[i].eta);
    iteration[py::str("linear_solver_iterations")] = py::int_(its[i].linear_solver_iterations);
    iterations.append(iteration);
  }
  summary_dict[py::str("iterations")] = iterations;
  summary_dict[py::str("brief_report")] = py::str(summary.BriefReport().c_str());
  summary_dict[py::str("full_report")] = py::str(summary.FullReport().c_str());
  return summary_dict;
}

PYBIND11_MODULE(_eris, m)
{
  py::class_<ceres::Solver::Summary>(m, "Summary");

  py::class_<eris::hand_eye_calibration::Solver>(m, "Solver")
      .def(py::init<const Eigen::Vector4d&, const Eigen::Vector3d&>())
      .def("add_residual_block", &eris::hand_eye_calibration::Solver::AddResidualBlock)
      .def("solve", &eris::hand_eye_calibration::Solver::Solve)
      .def("summary", &eris::hand_eye_calibration::Solver::Summary);

  py::class_<eris::hand_eye_calibration::laser2d::Solver>(m, "Laser2dSolver")
      .def(py::init<const Eigen::Vector4d&, const Eigen::Vector3d&, const Eigen::Vector4d&>())
      .def("add_residual_block", &eris::hand_eye_calibration::laser2d::Solver::AddResidualBlock)
      .def("solve", &eris::hand_eye_calibration::laser2d::Solver::Solve)
      .def("summary", &eris::hand_eye_calibration::laser2d::Solver::Summary);


  m.def("summary_to_dict", &SummaryToDict);
}