from abc import ABC, abstractmethod
from .models import LinearStateSpaceSystem
from .utils import is_callable, is_instance
import numpy as np
import numpy.typing as npt
from numbers import Number
from typing import Callable, Any

class Controller(ABC):
    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def output(self):
        pass

class PI(LinearStateSpaceSystem,Controller):

    def __init__(self, gains: npt.ArrayLike):
        self.gains = np.array(gains, dtype=np.float64)
        self.kp, self.ki = self.gains
        super().__init__(A=np.array(0),
                         B=np.array(1),
                         C=np.array(self.ki),
                         D=np.array(self.kp),
                         x0=np.array(0))

class PD(LinearStateSpaceSystem,Controller):

    def __init__(self, gains: npt.ArrayLike, tau: Number):
        self.tau = np.float64(tau)
        self.gains = np.array(gains, dtype=np.float64)
        self.kp, self.kd = self.gains
        super().__init__(A=np.array(-1/self.tau),
                         B=np.array(1),
                         C=np.array(self.kd/self.tau),
                         D=np.array(self.kp),
                         x0=np.array(0))

class FeedbackbLinearization(Controller):

    def __init__(self, ref: Callable[[np.float64, np.ndarray, Any], np.ndarray], 
        gains: npt.ArrayLike, 
        linearize: Callable[[np.float64, np.ndarray, np.ndarray, Any], np.ndarray], **kwargs):
        self.__dict__ = kwargs
        self.kwargs = kwargs
        is_callable(ref)
        self.ref = ref
        self.gains = np.array(gains)
        is_callable(linearize)
        self.linearize = linearize
    
    def output(self, t: Number, states: npt.ArrayLike) -> np.ndarray:
        states = np.array(states, dtype=np.float64)
        t=np.float64(t)
        ref = self.ref(t, states, **self.kwargs)
        error = ref[:-1]-states
        u = np.dot(self.gains, error) + ref[-1]
        y = self.linearize(t, u, states, **self.kwargs)
        return y