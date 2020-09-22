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

np.set_printoptions(suppress=True)
from glob import glob

import eris

from eris.transformations import quaternion_matrix, inverse_matrix

# Camera frame with respect to base
C = np.array(
    [
        [0.0, 1.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 1.2],
        [0.0, 0.0, -1.0, 1.4],
        [0.0, 0.0, 0.0, 1.0],
    ]
)

Cinv = inverse_matrix(C)

def random_pose():
    q = np.random.rand(4)
    q /= np.linalg.norm(q)
    T = quaternion_matrix(q)
    T[:3, 3] = np.random.rand(3)
    return T


def chessboard_corners(pattern_size=(10, 6), square_size=0.02):
    pattern_points = np.zeros((np.prod(pattern_size), 4), np.float64)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points[:, :2] *= square_size
    pattern_points[:, -1] += 1.0
    return pattern_points.T

P_chessboard = chessboard_corners()

X_ee_to_chessboard = random_pose()

num_samples = 10
robposes = [random_pose() for _ in range(num_samples)]
campoints = [(Cinv @ A @ X_ee_to_chessboard @ P_chessboard)[:3,:].T for A in robposes]


problem = eris.Problem(campoints, robposes)
solver = eris.Solver()

sol, summary = solver.calibrate_eye_to_hand(problem)

print(C)
print(sol)

print(C @ inverse_matrix(sol))

print(summary["brief_report"])
