#!/home/arog/anaconda2/bin/python

from DoubleIntegrator import *
from ConvexOptimalControl import *
import cvxpy as cvx
import numpy as np
import control as ctrl
import matplotlib.pyplot as plt

if __name__ == '__main__':
    dt = 0.01
    Nsim_low = 200
    Nsim_high = 400
    tspan = np.array([Nsim_low, Nsim_high])
    x0 = [0.0, -2.0]
    xf = [-1.0, 0.0]
    umax = 1.0
    umin = 0.0
    bcs = BoundaryConditions(x0,xf)
    ctrlConstraints = np.array([umin, umax])
    findFeasible = True

    db = DoubleIntegrator(2)
    db.makeStateSpaceSystem()
    db.makeDiscreteSystem(dt, 'zoh')

    mintime = ConvexOptimalControl(db, bcs, tspan, ctrlConstraints, findFeasible)
