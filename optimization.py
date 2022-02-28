from scipy.optimize import minimize
import numpy as np

from scipy.optimize import Bounds

bounds = Bounds([0,np.pi], [0, np.pi])

def eqn(x):
    return -(np.sin(x[0]) + np.sin(x[1]) - np.sin(x[0] + x[1]))

from scipy.optimize import LinearConstraint
linear_constraint = LinearConstraint([[1, 1]], [np.pi],  [2 * np.pi])

x0 = np.array([np.pi * 3 / 4, np.pi * 3 / 4])

res = minimize(eqn, x0, constraints=linear_constraint)
print(res)