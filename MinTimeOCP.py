from DoubleIntegrator import *
import numpy as np
import cvxpy as cvx
import control as ctrl
import matplotlib.pyplot as plt

def getOCP_Value(Nsim, dt, x0, xf, umax):
    db = DoubleIntegrator(2)
    db.makeStateSpaceSystem()
    db.makeDiscreteSystem(dt,'zoh')

    A = db.sysd.A
    B = db.sysd.B
    x = cvx.Variable(2, Nsim+1)
    u = cvx.Variable(2, Nsim)
    constr = []
    cost = 0
    # Solve the OCP the first time
    for t in range(Nsim):
        constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                   0 <= u[0,t], u[0,t] <= umax,
                   0 <= u[1,t], u[1,t] <= umax]
    constr += [x[0,Nsim] == xf[0], x[1,Nsim] == xf[1], x[0,0] == x0[0], x[1,0] == x0[1]]
    prob = cvx.Problem(cvx.Minimize(cost), constr)
    prob.solve(verbose = True)
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
            constr += [x[0,Nsim] == xf[0], x[1,Nsim] == xf[1], x[0,0] == x0[0], x[1,0] == x0[1]]
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
            constr += [x[0,Nsim] == xf[0], x[1,Nsim] == xf[1], x[0,0] == x0[0], x[1,0] == x0[1]]
            prob = cvx.Problem(cvx.Minimize(cost), constr)
            prob.solve(verbose = True)
        return Nsim

if __name__ == '__main__':

    dt = 0.01
    Nsim_try = 278
    x0 = [1.0,0.0]
    xf = [-1.0,0.0]
    umax = 1.0
    Nsim = getOCP_Value(Nsim_try, dt, x0, xf, umax)
    print "Optimal value for Nsim: " + str(Nsim)
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
    constr += [x[0,Nsim] == xf[0], x[1,Nsim] == xf[1], x[0,0] == x0[0], x[1,0] == x0[1]]
    prob = cvx.Problem(cvx.Minimize(cost), constr)
    prob.solve(verbose = True)

    U = u[0,:].value.A - u[1,:].value.A

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
