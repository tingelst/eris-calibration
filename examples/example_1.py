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

sol, summary = solver.calibrate_eye_to_hand(problem)

print(sol)

print(summary["full_report"])