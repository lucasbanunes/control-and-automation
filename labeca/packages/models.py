import numpy as np
from scipy import integrate


class DynamicSystem(object):

    def __init__(self):
        raise NotImplementedError

    def dx(self, t, x, u):
        raise NotImplementedError
    
    def output(self, t, u):
        raise NotImplementedError
    
    def update_state(self, t, u):
        raise NotImplementedError

class LinearStateSpaceSystem(DynamicSystem):

    def __init__(self, A, B, C, D, x0, t0, integrator='RK45'):
        self.A=A
        self.B=B
        self.C=C
        self.D=D
        if integrator != 'RK45':
            raise NotImplementedError
        self.integrator=getattr(integrate, integrator)(
            fun=self.dx
        )

    def dx(self, t, x, u):
        return self.A@x+self.B@u
    
    def output(self, t, u):
        return self.C@self.x+self.D@u
    
    def update_state(self, t, u):
        raise NotImplementedError

class PIController(LinearStateSpaceSystem):

    def __init__(self, kp, ki, integrator='RK45'):
        self.kp=kp
        self.ki=ki
        super().__init__(A=np.array([[0]]),
                         B=np.array([[1]]),
                         C=np.array([[self.ki]]),
                         D=np.array([[self.kp]]),
                         x0=np.array([[0]]),
                         integrator=integrator)