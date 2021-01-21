# Copyright 2021 Norwegian University of Science and Technology.
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
from numpy import random

np.set_printoptions(suppress=True)

from eris.transformations import (
    inverse_matrix,
    quaternion_from_matrix,
    quaternion_matrix,
    random_quaternion,
)
import _eris


def normalized(x):
    return x / np.linalg.norm(x)


def plane_plane_intersection(A, B):
    """
    Returns the intersection, a line, between the plane A and B
    - A and B are planes equations, such as A0 * x + A1 * y + A2 * z + A3 = 0
    - The line is returned as (U, V), where any point of the line is t * U + C, for all values of t
    - U is a normalized vector
    - C is the line origin, with the triangle (Ao, Bo, C) is orthogonal to the plane A and B,
        with Ao and Bo being the origin of plane A an B
    - If A and B are parallel, a numpy.linalg.LinAlgError exception is raised
    """
    U = normalized(np.cross(A[:-1], B[:-1]))
    M = np.array((A[:-1], B[:-1], U))
    X = np.array((-A[-1], -B[-1], 0.0))
    return U, np.linalg.solve(M, X)


def random_pose():
    q = np.random.rand(4)
    q /= np.linalg.norm(q)
    T = quaternion_matrix(q)
    T[:3, 3] = np.random.rand(3)
    return T


def random_plane():
    p = np.random.rand(4)
    p[:3] /= np.linalg.norm(p[:3])
    return p


# calibration_plane_in_base = [0, 0, 1, 0]
calibration_plane_in_base = random_plane()

X = random_pose()  # Laser in end-effector
Xinv = inverse_matrix(X)

num_samples = 100
robposes = [random_pose() for _ in range(num_samples)]
inverse_robposes = [inverse_matrix(A) for A in robposes]


# The laser lies in the XZ-plane of the sensor. Selects y as normal direction.
laser_planes_in_base = [(A @ X)[:, 1] for A in robposes]

laser_lines_in_calibration_plane = [
    plane_plane_intersection(calibration_plane_in_base, laser_plane_in_base)
    for laser_plane_in_base in laser_planes_in_base
]

ts = np.linspace(-1, 1)

laser_points_in_base = []
for laser_line in laser_lines_in_calibration_plane:
    points = []
    dir, point = laser_line
    for t in ts:
        points.append(np.array(t * dir + point))
    laser_points_in_base.append(points)

laser_points_in_sensor = []
for Ainv, ps in zip(inverse_robposes, laser_points_in_base):
    tmp = []
    for p in ps:
        tmp.append(
            Xinv[:3, :3] @ (Ainv[:3, :3] @ p.reshape(3, 1) + Ainv[:3, [3]])
            + Xinv[:3, [3]]
        )
    laser_points_in_sensor.append(tmp)


quat_init = np.roll(random_quaternion(), 1)
trs_init = np.random.rand(3)
plane_init = random_plane()

print(np.linalg.norm(plane_init[:3]))

solver = _eris.Laser2dSolver(quat_init, trs_init, plane_init)

for A, ps in zip(robposes, laser_points_in_sensor):
    q = np.roll(quaternion_from_matrix(A), 1)
    t = A[:3, 3]
    for p in ps:
        solver.add_residual_block(q, t, p)


qopt, topt, planeopt = solver.solve()

Xopt = quaternion_matrix(np.roll(qopt, -1))
Xopt[:3, 3] = topt

summary = _eris.summary_to_dict(solver.summary())
print(summary["brief_report"])

print(X)
print(Xopt)
print(Xopt @ Xinv)
print(planeopt)
print(calibration_plane_in_base)