import numpy as np

class DynamicSystem(object):

    def __init__(self):
        raise NotImplementedError

    def dx(self, t, x, u):
        raise NotImplementedError
    
    def __call__(self, t, x, u):
        return self.dx(t, x, u)
    
    def output(self, t, u):
        raise NotImplementedError

class LinearStateSpaceSystem(DynamicSystem):

    def __init__(self, A, B, C, D, x0):
        self.A=A
        self.B=B
        self.C=C
        self.D=D

    def dx(self, t, x, u):
        return self.A@x+self.B@u
    
    def output(self, t, u):
        return self.C@self.x+self.D@u