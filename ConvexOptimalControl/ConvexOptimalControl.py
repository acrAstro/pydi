import cvxpy as cvx
import control as ctrl
import numpy as np
from math import floor

class BoundaryConditions:
    def __init__(self, x0, xf):
        self.x0 = x0
        self.xf = xf
        self.validate()

    def validate(self):
        assert len(self.x0) == len(self.xf)

class ConvexOptimalControl:
    def __init__(self, stateSpaceSystem, boundaryConditions, tspan, ctrlConstraints, findFeasible):
        # stateSpaceSystem is a State Space object
        self.stateSpaceSystem = stateSpaceSystem
        # x0 and xf are members of the BoundaryConditions object
        self.x0 = boundaryConditions.x0
        self.xf = boundaryConditions.xf
        # tspan is the bracket for searching for a feasible maneuver time
        self.tmin = tspan[0]
        self.tmax = tspan[1]
        # Set control constraints
        self.umin = ctrlConstraints[0]
        self.umax = ctrlConstraints[1]
        # Maybe we don't want to automatically do the line search when this
        # class is instantiated, so set findFeasible to False if that's the case
        if (findFeasible):
            self.findFeasibleTime()
        else:
            pass

    def findFeasibleTime(self):
        tol = self.stateSpaceSystem.dt
        counter = 0
        maxEval = 1e3
        XR = self.tmax
        XL = self.tmin
        # Test the OCP at the boundaries
        for Nsim in [self.tmin, self.tmax]:
            prob = self.tryOCP(int(Nsim))
            assert (prob.status != 'infeasible'), 'No feasible solution in [XL, XR]'
        # Bisect the positive real number line to find a feasible solution
        while (abs(XR - XL) > tol):
            counter += 1
            if (counter > maxEval):
                print 'Maximum number of functions evaluations exceeded!'
                break
#            Nsim = (XL+XR)/2
 #           prob = self.tryOCP(int(Nsim))
            if (prob.status == 'infeasible'):
                XL = floor((XL+XR)/2)
                Nsim  = XL
                prob = self.tryOCP(int(Nsim))
            elif (prob.status == 'optimal'):
                XR =  floor((XL+XR)/2)
                Nsim = XR
                prob = self.tryOCP(int(Nsim))
        self.NsimMintime = Nsim
        self.probMinTime = self.tryOCP(int(self.NsimMintime))
        
    def tryOCP(self, Nsim):
        x = cvx.Variable(len(self.x0), Nsim+1)
        u = cvx.Variable(self.stateSpaceSystem.B[0,:].size, Nsim)
        constr = []
        cost = 0
        A = self.stateSpaceSystem.A
        B = self.stateSpaceSystem.B
        print "Trying " + str(Nsim)
        for t in range(Nsim):
            constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                       0 <= u[0,t], u[0,t] <= self.umax,
                       0 <= u[1,t], u[1,t] <= self.umax]
        constr += [x[:,Nsim] == self.xf, x[:,0] == self.x0]
        prob = cvx.Problem(cvx.Minimize(cost), constr)
        prob.solve(verbose = True)
        return prob
