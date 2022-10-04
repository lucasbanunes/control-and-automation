from .models import LinearStateSpaceSystem
import numpy as np

class PIController(LinearStateSpaceSystem):

    def __init__(self, kp, ki):
        self.kp=kp
        self.ki=ki
        super().__init__(A=np.array([[0]]),
                         B=np.array([[1]]),
                         C=np.array([[self.ki]]),
                         D=np.array([[self.kp]]),
                         x0=np.array([[0]]))

