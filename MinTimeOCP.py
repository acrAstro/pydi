from DoubleIntegrator import *
import numpy as np
import cvxpy as cvx
import control as ctrl
import matplotlib.pyplot as plt

# Written by: AC Rogers, 2016-Oct-14

def getOCP_Value(Nsim, dt, x0, xf, umax):
    '''
    This function determines the value of Nsim for which the minimum-time,
    double integrator constraint satisfaction problem is satisfied.

    INPUTS:
            Nsim : Number of simulation steps
            dt   : Time step for discretizing the linear system
            x0   : Initial conditions for maneuver
            xf   : Terminal conditions for maneuver
            umax : Maximum control force that can be exerted

    OUTPUT:
            Nsim : Value of Nsim for which the constraint satisfaction problem
                   has a valid, optimal solution
    '''
    # Create an instance of the double integrator
    db = DoubleIntegrator(2)
    # Make the state space matrices (we only need A and B)
    db.makeStateSpaceSystem()
    # Discretize it
    db.makeDiscreteSystem(dt,'zoh')
    A = db.sysd.A
    B = db.sysd.B

    # Make the cvxpy environment variables
    x = cvx.Variable(2, Nsim+1)
    u = cvx.Variable(2, Nsim)

    # Initialize an empty array of constraints
    constr = []

    # The cost needs to be zero for minimum time-of-flight
    cost = 0
    # Solve the OCP the first time
    for t in range(Nsim):
        # Build the problem constraints
        constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                   0 <= u[0,t], u[0,t] <= umax,
                   0 <= u[1,t], u[1,t] <= umax]

    # Set the boundary conditions
    constr += [x[:,Nsim] == xf, x[:,0] == x0]
    # Create the problem
    prob = cvx.Problem(cvx.Minimize(cost), constr)

    ####### SOLVE THE PROBLEM ON THIS LINE ###############
    prob.solve(verbose = True)

    # Based on the optimality/ infeasibility of the first try, sweep the real
    # number line (integers only) to find a suitable time-of-flight. If the
    # first solution was numerically optimal, the sweep decrements Nsim until it
    # is no longer feasible then adds one as the optimal time-of-flight. If the
    # problem is initially infeasible, it increments Nsim until a feasible
    # solution is found, implying that the constraints are satisfied.
    if (prob.status == 'optimal'):
        while (prob.status == 'optimal'):
            Nsim -= 1
            x = cvx.Variable(2, Nsim+1)
            u = cvx.Variable(2, Nsim)
            constr = []
            cost = 0
            print "Trying " + str(Nsim)
            for t in range(Nsim):
                constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                           0 <= u[0,t], u[0,t] <= umax,
                           0 <= u[1,t], u[1,t] <= umax]
            constr += [x[:,Nsim] == xf, x[:,0] == x0]
            prob = cvx.Problem(cvx.Minimize(cost), constr)
            prob.solve(verbose = True)
        return Nsim+1
    elif (prob.status == 'infeasible'):
        while (prob.status == 'infeasible'):
            Nsim += 1
            x = cvx.Variable(2, Nsim+1)
            u = cvx.Variable(2, Nsim)
            constr = []
            cost = 0
            print "Trying " + str(Nsim)
            for t in range(Nsim):
                constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                           0 <= u[0,t], u[0,t] <= umax,
                           0 <= u[1,t], u[1,t] <= umax]
            constr += [x[:,Nsim] == xf, x[:,0] == x0]
            prob = cvx.Problem(cvx.Minimize(cost), constr)
            prob.solve(verbose = True)
        return Nsim

if __name__ == '__main__':

    # Input parameters
    # Time step
    dt = 0.01
    # Initial guess for solver
    Nsim_try = 278
    # Initial conditions
    x0 = [1.0,0.0]
    # Terminal conditions
    xf = [-1.0,0.0]
    # Control constraint, u[ii] may never exceed this value
    umax = 1.0

    # Find the optimal value of Nsim to satisfy constraints
    Nsim = getOCP_Value(Nsim_try, dt, x0, xf, umax)
    print "Optimal value for Nsim: " + str(Nsim)

    # Now that we've found the optimal value of Nsim, solve the OCP one more
    # time with the optimal value, this is the final solution
    db = DoubleIntegrator(2)
    db.makeStateSpaceSystem()
    db.makeDiscreteSystem(dt,'zoh')
    A = db.sysd.A
    B = db.sysd.B
    x = cvx.Variable(2, Nsim+1)
    u = cvx.Variable(2, Nsim)
    constr = []
    cost = 0
    for t in range(Nsim):
        constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                   0 <= u[0,t], u[0,t] <= umax,
                   0 <= u[1,t], u[1,t] <= umax]
    constr += [x[:,Nsim] == xf, x[:,0] == x0]
    prob = cvx.Problem(cvx.Minimize(cost), constr)
    prob.solve(verbose = True)

    # Since (for this problem) we had control going strictly positive and
    # negative, this aggregates the two control signals into one single signal
    # to plot.
    U = u[0,:].value.A - u[1,:].value.A

    # Plot results.
    f = plt.figure()
    plt.subplot(221)
    plt.plot(x[0,:].value.A.flatten())
    plt.subplot(222)
    plt.plot(x[1,:].value.A.flatten())
    plt.subplot(223)
    plt.plot(U.flatten())
    plt.subplot(224)
    plt.plot(x[0,:].value.A.flatten(), x[1,:].value.A.flatten())
    plt.show()
