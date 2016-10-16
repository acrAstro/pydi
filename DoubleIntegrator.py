import numpy as np
import control as ctrl

class DoubleIntegrator:
    def __init__(self, nu):
        self.A = np.matrix([[0,1],[0,0]])
        # Note, this matrix doesn't really do anything right now
        self.C = np.matrix([[1,0],[0,1]])
        try:
            if nu == 1:
                self.B = np.matrix([[0],[1]])
                self.D = np.matrix([[0],[1]])
                return
            elif nu == 2:
                self.B = np.matrix([[0,0],[1,-1]])
                self.D = np.matrix([[0,0],[1,-1]])
                return
            elif nu != 1 or nu != 2:
                raise InputError("ERROR: Input 'nu' must be either nu = 1 or nu = 2")
        except InputError as me:
            print me.message
            return

    def makeStateSpaceSystem(self):
        self.sysc = ctrl.matlab.ss(self.A, self.B, self.C, self.D)

    def makeDiscreteSystem(self, dt, method):
        self.sysd = ctrl.matlab.c2d(self.sysc, dt, method)

class Error(Exception):
    pass

class InputError(Error):
    def __init__(self, message):
        self.message = message

if __name__ == '__main__':
    db = DoubleIntegrator(2)
    db.makeStateSpaceSystem()
    db.makeDiscreteSystem(0.1, 'zoh')
    print db.sysd.B[0,:].size
