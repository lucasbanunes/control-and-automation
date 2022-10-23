from abc import ABC, abstractmethod
from .models import DynamicSystem, LinearStateSpaceSystem
from .utils import is_callable, is_instance
import numpy as np
import numpy.typing as npt
from numbers import Number
from typing import Callable, Any, Dict

def get_ctrl_from_config(config_dict: Dict[str, Any]):
    locals_dict = locals()
    controllers = [locals_dict[ctrl_class](**ctrl_config) 
        for ctrl_class, ctrl_config in config_dict.items()]
    return controllers

class Controller(DynamicSystem):
    def output(self, t: Number, input_x: npt.ArrayLike, 
        x: npt.ArrayLike, refs: npt.ArrayLike, **kwargs) -> np.ndarray:
        return super().output(t, x, refs-input_x)

    def dx(self, t: Number, input_x: npt.ArrayLike, 
        x: npt.ArrayLike, refs: npt.ArrayLike, **kwargs) -> np.ndarray:
        return super().dx(t, x, refs-input_x)

class LinearController(LinearStateSpaceSystem,Controller):
    def output(self, t: Number, input_x: npt.ArrayLike, 
        x: npt.ArrayLike, refs: npt.ArrayLike, **kwargs) -> np.ndarray:
        return super().output(t, x, refs-input_x)

    def dx(self, t: Number, input_x: npt.ArrayLike, 
        x: npt.ArrayLike, refs: npt.ArrayLike, **kwargs) -> np.ndarray:
        return super().dx(t, x, refs-input_x)


class PCtrl(LinearController):

    def __init__(self, gains: npt.ArrayLike, x0: npt.ArrayLike):
        self.gains = np.array(gains, dtype=np.float64)
        self.kp = self.gains[0]
        super().__init__(A=np.array([[0]]),
                         B=np.array([[0]]),
                         C=np.array([[0]]),
                         D=np.array([[self.kp]]),
                         x0=np.array(x0).reshape(-1,1))

class PICtrl(LinearController):

    def __init__(self, gains: npt.ArrayLike, x0: npt.ArrayLike):
        self.gains = np.array(gains, dtype=np.float64)
        self.kp, self.ki = self.gains
        super().__init__(A=np.array([[0]]),
                         B=np.array([[1]]),
                         C=np.array([[self.ki]]),
                         D=np.array([[self.kp]]),
                         x0=np.array(x0).reshape(-1,1))

class PDCtrl(LinearController):

    def __init__(self, gains: npt.ArrayLike, tau: Number, x0: npt.ArrayLike):
        self.tau = np.float64(tau)
        self.gains = np.array(gains, dtype=np.float64)
        self.kp, self.kd = self.gains
        super().__init__(A=np.array([[-1/self.tau]]),
                         B=np.array([[1]]),
                         C=np.array([[self.kd/self.tau]]),
                         D=np.array([[self.kp]]),
                         x0=np.array(x0).reshape(-1,1))

class ExplicitCtrl(Controller):
    def __init__(self, gains:npt.ArrayLike):
        self.gains = np.array(gains, dtype=np.float64)
    
    def output(self, t: Number, refs:npt.ArrayLike, input_x: npt.ArrayLike, **kwargs) -> np.ndarray:
        input_x = np.array(input_x, dtype=np.float64)
        refs = np.array(refs, dtype=np.float64)
        t=np.float64(t)
        errors = refs[:-1]-input_x
        u = np.dot(self.gains, errors) + refs[-1]
    
    def dx(self, t: Number, refs:npt.ArrayLike, input_x: npt.ArrayLike, **kwargs) -> np.ndarray:
        return np.array([0], dtype=np.float64)

class FbLinearizationCtrl(Controller):

    def __init__(self, gains: npt.ArrayLike, controller: Controller, **kwargs):
        self.__dict__ = kwargs
        self.kwargs = kwargs
        self.gains = np.array(gains, dtype=np.float64)
        if isinstance(controller, Controller):
            self.controller = controller
        elif isinstance(controller, dict):
            self.controller = get_ctrl_from_config(controller)
        else:
            raise TypeError('controller can only be a config dict or a Controller instance')
    
    def output(self, t: Number, refs:npt.ArrayLike, input_x: npt.ArrayLike, **kwargs) -> np.ndarray:
        u = self.controller.output(t, refs, input_x, **kwargs)
        y = self.linearize(t, u, input_x, **self.kwargs, **kwargs)
        return y
    
    def dx(self, t: Number, refs:npt.ArrayLike, input_x: npt.ArrayLike, **kwargs) -> np.ndarray:
        return self.controller.dx(t, refs, input_x, **kwargs)
    
    @abstractmethod
    def linearize(self, t: Number, u: np.ndarray, input_x: np.ndarray, **kwargs) -> np.ndarray:
        pass
        