#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <eris/solver.hpp>

namespace py = pybind11;

PYBIND11_MODULE(_eris, m)
{
  py::class_<eris::hand_eye_calibration::Solver>(m, "Solver")
      .def(py::init<const Eigen::Vector4d&, const Eigen::Vector3d&>())
      .def("add_residual_block", &eris::hand_eye_calibration::Solver::AddResidualBlock)
      .def("solve", &eris::hand_eye_calibration::Solver::Solve);
  //   .def("summary", &eris::hand_eye_calibration::Summary);
}