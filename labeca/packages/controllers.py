from abc import ABC, abstractmethod
from .models import DynamicSystem, LinearStateSpaceSystem
from .utils import is_callable, is_instance
import numpy as np
import numpy.typing as npt
from numbers import Number
from typing import Callable, Any, Dict

class Controller(DynamicSystem):
    pass

class LinearController(LinearStateSpaceSystem,Controller):
    def output(self, t: Number, x: npt.ArrayLike, refs: npt.ArrayLike) -> np.ndarray:
        return super().output(t, x, refs-x)

    def dx(self, t: Number, x: npt.ArrayLike, refs: npt.ArrayLike) -> np.ndarray:
        return super().dx(t, x, refs-x)


class PCtrl(LinearController):

    def __init__(self, gains: npt.ArrayLike):
        self.gains = np.array(gains, dtype=np.float64)
        self.kp = self.gains[0]
        super().__init__(A=np.array([[0]]),
                         B=np.array([[0]]),
                         C=np.array([[0]]),
                         D=np.array([[self.kp]]),
                         x0=np.array([[0]]))

    def output(self, t: Number, x: npt.ArrayLike, refs: npt.ArrayLike) -> np.ndarray:
        return super().output(t, x, refs-x)

    def dx(self, t: Number, x: npt.ArrayLike, refs: npt.ArrayLike) -> np.ndarray:
        return super().dx(t, x, refs-x)

class PICtrl(LinearController):

    def __init__(self, gains: npt.ArrayLike):
        self.gains = np.array(gains, dtype=np.float64)
        self.kp, self.ki = self.gains
        super().__init__(A=np.array([[0]]),
                         B=np.array([[1]]),
                         C=np.array([[self.ki]]),
                         D=np.array([[self.kp]]),
                         x0=np.array([[0]]))

class PDCtrl(LinearController):

    def __init__(self, gains: npt.ArrayLike, tau: Number):
        self.tau = np.float64(tau)
        self.gains = np.array(gains, dtype=np.float64)
        self.kp, self.kd = self.gains
        super().__init__(A=np.array([[-1/self.tau]]),
                         B=np.array([[1]]),
                         C=np.array([[self.kd/self.tau]]),
                         D=np.array([[self.kp]]),
                         x0=np.array([[0]]))

class ExplicitCtrl(Controller):
    def __init__(self, gains:npt.ArrayLike):
        self.gains = np.array(gains, dtype=np.float64)
    
    def output(self, t: Number, refs:npt.ArrayLike, states: npt.ArrayLike, **extra) -> np.ndarray:
        states = np.array(states, dtype=np.float64)
        refs = np.array(refs, dtype=np.float64)
        t=np.float64(t)
        errors = refs[:-1]-states
        u = np.dot(self.gains, errors) + refs[-1]
    
    def dx(self, t: Number, refs:npt.ArrayLike, states: npt.ArrayLike, **extra) -> np.ndarray:
        return np.array([0], dtype=np.float64)

class FbLinearizationCtrl(Controller):

    def __init__(self, gains: npt.ArrayLike, controller: Controller, **kwargs):
        self.__dict__ = kwargs
        self.kwargs = kwargs
        self.gains = np.array(gains, dtype=np.float64)
        is_instance(controller, Controller)
        self.controller = controller
    
    def output(self, t: Number, refs:npt.ArrayLike, states: npt.ArrayLike, **extra) -> np.ndarray:
        u = self.controller.output(t, refs, states, **extra)
        y = self.linearize(t, u, states, **self.kwargs, **extra)
        return y
    
    def dx(self, t: Number, refs:npt.ArrayLike, states: npt.ArrayLike, **extra) -> np.ndarray:
        return self.controller.dx(t, refs, states, **extra)
    
    @abstractmethod
    def linearize(self, t: Number, u: np.ndarray, states: np.ndarray) -> np.ndarray:
        pass
        