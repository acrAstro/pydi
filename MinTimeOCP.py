from DoubleIntegrator import *
import numpy as np
import cvxpy as cvx
import control as ctrl
import matplotlib.pyplot as plt


db = DoubleIntegrator(2)
db.makeStateSpaceSystem()
db.makeDiscreteSystem(0.1,'zoh')

A = db.sysd.A
B = db.sysd.B

Nsim = 20

x = cvx.Variable(2, Nsim+1)
u = cvx.Variable(2, Nsim)
x0 = [1.0,0.0]
xf = [0.0,0.0]
umax = 1.0

constr = []
cost = 0

for t in range(Nsim):
    cost += cvx.norm(u[:,t],1)
    constr += [x[:,t+1] == A*x[:,t] + B*u[:,t],
                   0 <= u[0,t], u[0,t] <= umax,
                   0 <= u[1,t], u[1,t] <= umax]
    
constr += [x[0,Nsim] == xf[0], x[1,Nsim] == xf[1], x[0,0] == x0[0], x[1,0] == x0[1]]
prob = cvx.Problem(cvx.Minimize(cost), constr)
prob.solve(verbose = True)

print prob.objective.value

U = u[0,:].value.A - u[1,:].value.A

f = plt.figure()
plt.subplot(311)
plt.plot(x[0,:].value.A.flatten())
plt.subplot(312)
plt.plot(x[1,:].value.A.flatten())
plt.subplot(313)
plt.plot(U.flatten())

plt.show()
