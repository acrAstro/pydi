import cvxpy as cvx
import control as ctrl
import numpy as np

class BoundaryConditions:
    def __init__(self, x0, xf):
        self.x0 = x0
        self.xf = xf
        self.validate()

    def validate(self):
        assert len(x0) == len(xf)

class ConvexOptimalControl:
    def __init__(self, stateSpaceSystem, boundaryConditions, tspan, findFeasible):
        # stateSpaceSystem is a State Space object
        self.stateSpaceSystem = stateSpaceSystem
        # x0 and xf are members of the BoundaryConditions object
        self.x0 = boundaryConditions.x0
        self.xf = boundaryConditions.xf
        # tspan is the bracket for searching for a feasible maneuver time
        self.tmin = tspan[0]
        self.tmax = tspan[1]
        # Maybe we don't want to automatically do the line search when this
        # class is instantiated, so set findFeasible to False if that's the case
        if (findFeasible):
            self.findFeasibleTime()
        else:
            pass

    def findFeasibleTime(self):
