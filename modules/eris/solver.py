# Copyright 2020 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import numpy as np

import _eris

from eris.problem import Problem
from eris.transformations import (
    quaternion_from_matrix,
    quaternion_matrix,
    inverse_matrix,
)


class Solver:
    def __init__(self):
        pass

    def _solve(self, problem: Problem, x=None):
        """
        Solve the given problem (starting from a 'random' initial guess if the optional argument is not provided).
        """
        if x is not None:
            q0, t0 = x
        else:
            q0 = np.array([1.0, 0.0, 0.0, 0.0])
            t0 = np.array([0.0, 0.0, 0.0])

        campoints = problem.campoints
        robposes = problem.robposes

        solver = _eris.Solver(q0, t0)

        n = len(robposes)
        for i in range(n):
            for j in range(i + 1, n):
                Ti = robposes[i]
                qi = np.roll(quaternion_from_matrix(Ti), 1)
                ti = Ti[:3, 3]
                pis = campoints[i]

                Tj = robposes[j]
                qj = np.roll(quaternion_from_matrix(Tj), 1)
                tj = Tj[:3, 3]
                pjs = campoints[j]

                for pi, pj in zip(pis, pjs):
                    solver.add_residual_block(qi, ti, pi, qj, tj, pj)

        qopt, topt = solver.solve()
        Xopt = quaternion_matrix(np.roll(qopt, -1))
        Xopt[:3, 3] = topt

        summary = _eris.summary_to_dict(solver.summary())

        return Xopt, summary

    def calibrate_eye_in_hand(self, problem, x=None):
        return self._solve(problem, x)

    def calibrate_eye_to_hand(self, problem, x=None):
        problem.robposes = [inverse_matrix(robpose) for robpose in problem.robposes]
        return self._solve(problem, x)