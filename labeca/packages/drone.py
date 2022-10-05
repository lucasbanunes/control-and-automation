import numpy as np
import numpy.typing as npt
from packages.models import DynamicSystem
from packages.utils import is_callable, is_numeric, is_instance
from typing import Callable
from numbers import Number

states_names = ['phi','dphi','theta','dtheta','psi','dpsi','x','dx','y','dy','z','dz']

class DroneController():

    def __init__(self, ref_x: Callable[[Number],Number], ref_y: Callable[[Number],Number], 
        ref_z: Callable[[Number],Number], ref_dz: Callable[[Number],Number], ref_ddz: Callable[[Number],Number], 
        ref_psi: Callable[[Number],Number], kpz: Number, kdz: Number, g: Number, mass: Number,
        A: npt.ArrayLike, mx: Number, my: Number, mz: Number):
        """
        Parameters
        ------------
        ref_x: callable
            Callable that recieves time and returns desired x value
        ref_y: callable
            Callable that recieves time and returns desired y value
        ref_z: callable
            Callable that recieves time and returns desired z value
        ref_dz: callable
            Callable that recieves time and returns desired dz value
        ref_ddz: callable
            Callable that recieves time and returns desired ddz value
        ref_psi: callable
            Callable that recieves time and returns desired psi value
        kpz: Number
            Proportional gain for z control
        kdz: Number
            Derivative gain for z control
        g: Number
            Gravity
        mass: Number
            Drone mass
        A: Array like
            Square matrix for computing per motor thurst
        
        These 3 bellow will have their values computed by the controller
        mx: Number
            x moment
        my: Number
            y moment
        mz: Number
            z moment
        """
    
        is_callable(ref_x)
        self.ref_x = ref_x
        is_callable(ref_y)
        self.ref_y = ref_y
        is_callable(ref_z)
        self.ref_z = ref_z
        is_callable(ref_dz)
        self.ref_dz = ref_dz
        is_callable(ref_ddz)
        self.ref_ddz = ref_ddz
        is_callable(ref_psi)
        self.ref_psi = ref_psi
        is_numeric(kpz)
        self.kpz = np.float64(kpz)
        is_numeric(kdz)
        self.kdz = np.float64(kdz)
        is_numeric(g)
        self.g = np.float64(g)
        is_numeric(mass)
        self.mass = np.float64(mass)
        A = np.array(A, dtype=np.float64)
        self.Ainv = np.linalg.inv(A)
        is_numeric(mx)
        self.mx = mx
        is_numeric(my)
        self.my = my
        is_numeric(mz)
        self.mz = mz
    
    def __call__(self, t: Number, xs: npt.ArrayLike) -> np.ndarray:

        # phi, dphi, theta, dtheta, psi, dpsi, x, dx, y, dy, z, dz = xs
        phi, _, theta, _, _, _, _, _, _, _, z, dz = xs
        ref_z = self.ref_z(t)
        ref_dz = self.ref_dz(t)
        ref_ddz = self.ref_ddz(t)
        ez = ref_z - z
        dez = ref_dz - dz
        uz = self.kpz*ez + self.kdz*dez + ref_ddz
        f = (uz+self.g)*self.mass/(np.cos(phi)*np.cos(theta))

        # Add mx, my, mz computation

        fi = np.dot(self.Ainv, np.array((f, self.mx, self.my, self.mz)))
        return fi
        
class Drone(DynamicSystem):

    def __init__(self, jx: Number, jy: Number, jz: Number,
        g: Number, mass: Number, A:npt.ArrayLike):
        """
        Parameters
        ------------
        jx: Number
            Drone x inertia
        jy: Numver
            Drone y inertia
        jz: Number
            Drone z inertia
        g: Number
            Gravity
        mass: Number
            Drone mass
        A: Array like
            Square matrix for computing per motor thurst
        """
        is_numeric(jx)
        self.jx = jx
        is_numeric(jy)
        self.jy = jy
        is_numeric(jz)
        self.jz = jz
        # is_numeric(mx)
        # self.mx = mx
        # is_numeric(my)
        # self.my = my
        # is_numeric(mz)
        # self.mz = mz
        is_numeric(jx)
        self.jx = jx
        is_numeric(g)
        self.g = g
        is_numeric(mass)
        self.mass = mass
        self.A = np.array(A, dtype=np.float64)
    
    def __call__(self, t: Number, xs: npt.ArrayLike, 
        fi: npt.ArrayLike) -> np.ndarray:
        f, mx, my, mz = np.dot(self.A, fi)
        f_over_m = f/self.mass
        phi, dphi, theta, dtheta, psi, dpsi, x, dx, y, dy, z, dz = xs
        sphi = np.sin(phi)
        cphi = np.cos(phi)
        stheta = np.sin(theta)
        ctheta = np.cos(theta)
        spsi = np.sin(psi)
        cpsi = np.cos(psi)
        
        ddphi = dtheta*dpsi*((self.jy-self.jz)/self.jx) + mx/self.jx
        ddtheta = dphi*dpsi*((self.jz-self.jx)/self.jy) + my/self.jy
        ddpsi = dtheta*dphi*((self.jx-self.jy)/self.jz) +  mz/self.jz
        ddx = f_over_m*((spsi*sphi) + (cphi*stheta*cpsi))
        ddy = f_over_m*((-cpsi*sphi) + (spsi*stheta*cphi))
        ddz = -self.g+(cphi*ctheta*f_over_m)

        dxs = np.array([dphi, ddphi, dtheta, ddtheta, dpsi, ddpsi, dx, ddx, dy, ddy, dz, ddz])
        return dxs

    def output(self, t, xs):
        phi, dphi, theta, dtheta, psi, dpsi, x, dx, y, dy, z, dz = xs
        return np.array([phi, theta, psi, x, y, z])

class Propeller(DynamicSystem):

    def __init__(self, c_tau: Number, kt: Number):
        """
        Parameters
        ------------
        c_tau: Number
            aerodynamic torque constant
        
        kt: Number
            thrust aerodynamic constant
        """
    

class ControledDrone(DynamicSystem):

    def __init__(self, controler: DroneController, drone: Drone) -> None:
        is_instance(controler, DroneController)
        self.controler=controler
        is_instance(drone, Drone)
        self.drone=drone

    def __call__(self, t, xs):
        controler_out = self.controler(t, xs)
        drone_dxs = self.drone(t, xs, controler_out)
        return drone_dxs
    
    def output(self, t, xs):
        phi, dphi, theta, dtheta, psi, dpsi, x, dx, y, dy, z, dz = xs
        return np.array([phi, theta, psi, x, y, z])