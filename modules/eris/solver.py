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


class Solver:
    def __init__(self):
        pass

    def solve(self, problem: Problem, x=None):
        """
        Solve the given problem (starting from a random initial guess if the optional argument is not provided).
        """
        if x is not None:
            q0, t0 = x
        else:
            q0 = np.array([1.0, 0.0, 0.0, 0.0])
            t0 = np.array([0.0,0.0,0.0])
        solver = _eris.Solver(q0, t0)

        return True
