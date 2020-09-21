import numpy as np
import eris

problem = eris.Problem()


solver = eris.Solver(q0, t0)

sol = solver.solve(problem)

print(sol)