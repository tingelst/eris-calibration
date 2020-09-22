import numpy as np

np.set_printoptions(suppress=True)
from glob import glob

import eris

from eris.transformations import quaternion_matrix

def inverse_matrix(A):
    R = A[:3, :3]
    t = A[:3, [3]]
    Ainv = np.eye(4)
    Ainv[:3, :3] = R.T
    Ainv[:3, [3]] = -R.T @ t
    return Ainv


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

P = chessboard_corners() # Points in chessboard
T = random_pose() # Chessboard in base
X = random_pose() # Camera in end-effector
Xinv = inverse_matrix(X)

num_samples = 10
robposes = [random_pose() for _ in range(num_samples)]
campoints = [(Xinv @ inverse_matrix(A) @ T @  P)[:3,:].T for A in robposes]


problem = eris.Problem(campoints, robposes)
solver = eris.Solver()

sol, summary = solver.calibrate_eye_in_hand(problem)

print(summary["brief_report"])
print(X)
print(sol)

print(X@inverse_matrix(sol))
