import numpy as np
import numpy.typing as npt
from abc import ABC, abstractmethod
from numbers import Number

class DynamicSystem(ABC):

    @abstractmethod
    def __init__(self):
        pass
    
    @abstractmethod
    def __call__(self) -> np.ndarray:
        pass
    
    @abstractmethod
    def output(self) -> np.ndarray:
        pass

class LinearStateSpaceSystem(DynamicSystem):

    def __init__(self, A: npt.ArrayLike, 
        B: npt.ArrayLike, C: npt.ArrayLike, 
        D: npt.ArrayLike, x0: npt.ArrayLike):
        self.A=np.array(A, np.float64)
        self.B=np.array(B, np.float64)
        self.C=np.array(C, np.float64)
        self.D=np.array(D, np.float64)
        self.x0=np.array(x0, np.float64)

    def __call__(self, t: Number, x: npt.ArrayLike, 
        u: npt.ArrayLike) -> np.ndarray:
        return np.dot(self.A, x)+np.dot(self.B, u)
    
    def output(self, t: Number, x: npt.ArrayLike, 
        u: npt.ArrayLike) -> np.ndarray:
        return np.dot(self.C, x)+np.dot(self.D, u)