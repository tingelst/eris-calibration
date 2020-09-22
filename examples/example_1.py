import numpy as np

np.set_printoptions(suppress=True)
from glob import glob

import eris

dataset_path = "/workspaces/eris/datasets/example_1"

robposes = [
    np.loadtxt(fname, delimiter=",")
    for fname in sorted(glob(dataset_path + "/robpos??.txt"))
]

campoints = [
    np.loadtxt(fname, delimiter=",")
    for fname in sorted(glob(dataset_path + "/campoints??.txt"))
]

problem = eris.Problem(campoints, robposes)


solver = eris.Solver()

sol, summary = solver.solve(problem)

print(sol)

a = np.loadtxt("/workspaces/eris/datasets/example_1/2019-10-21_20-50-11.txt")
# a[:3,3] *= 0.001

print(a)

def inv(A):
    R = A[:3, :3]
    t = A[:3, [3]]
    Ainv = np.eye(4)
    Ainv[:3, :3] = R.T
    Ainv[:3, [3]] = -R.T @ t
    return Ainv

print(sol @ inv(a))

print(summary['full_report'])