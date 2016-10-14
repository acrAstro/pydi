from DoubleIntegrator import *
import numpy as np
import cvxpy as cvx
import control as ctrl
import matplotlib.pyplot as plt

#def getOCP_Value(Nsim, boundaryConditions, ctrlConstraint):
    


db = DoubleIntegrator(2)
db.makeStateSpaceSystem()
db.makeDiscreteSystem(0.01,'zoh')

A = db.sysd.A
B = db.sysd.B

Nsim = 283

x = cvx.Variable(2, Nsim+1)
u = cvx.Variable(2, Nsim)
x0 = [1.0,0.0]
xf = [-1.0,0.0]
umax = 1.0

constr = []
cost = 0

for t in range(Nsim):
    constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                   0 <= u[0,t], u[0,t] <= umax,
                   0 <= u[1,t], u[1,t] <= umax]
    
constr += [x[0,Nsim] == xf[0], x[1,Nsim] == xf[1], x[0,0] == x0[0], x[1,0] == x0[1]]
prob = cvx.Problem(cvx.Minimize(0), constr)
prob.solve(verbose = True)

print prob.status == 'infeasible'

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
