import numpy as np
import pandas as pd
import numpy.typing as npt
from packages.controllers import Controller
from packages.models import DynamicSystem
from packages.utils import is_callable, is_numeric, is_instance
from typing import Callable, Union
from numbers import Number
from autograd import elementwise_grad as egrad
import autograd.numpy as anp
from collections import defaultdict

states_names = ['phi','dphi','theta','dtheta','psi','dpsi','x','dx','y','dy','z','dz']

class DroneController(Controller):

    def __init__(self, ref_x: Callable[[Number],Number], ref_y: Callable[[Number],Number], 
        ref_z: Callable[[Number],Number], ref_dz: Callable[[Number],Number], ref_ddz: Callable[[Number],Number], 
        ref_psi: Callable[[Number],Number], ref_dpsi: Callable[[Number],Number], ref_ddpsi: Callable[[Number],Number], 
        kp_z: Number, kd_z: Number, kp_phi:Number, kd_phi:Number, kp_theta:Number, kd_theta:Number, kp_psi:Number, kd_psi:Number,
        A: npt.ArrayLike, jx: Number, jy: Number, jz: Number,
        g: Number, mass: Number, log_internals:bool=False):
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
        ref_dpsi: callable
            Callable that recieves time and returns desired dpsi value
        ref_ddpsi: callable
            Callable that recieves time and returns desired ddpsi value
        kp_z: Number
            Proportional gain for z control
        kd_z: Number
            Derivative gain for z control
        kp_phi: Number
            Proportional gain for z control
        kd_phi: Number
            Derivative gain for z control
        kp_theta: Number
            Proportional gain for z control
        kd_theta: Number
            Derivative gain for z control
        kp_psi: Number
            Proportional gain for z control
        kd_psi: Number
            Derivative gain for z control
        g: Number
            Gravity
        mass: Number
            Drone mass
        A: Array like
            Square matrix for computing per motor thurst
        jx: Number
            Drone x inertia
        jy: Numver
            Drone y inertia
        jz: Number
            Drone z inertia
        log_internals: bool
            If true logs the internal variable values each time a output is computed by the controller
        """

        # References
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
        is_callable(ref_dpsi)
        self.ref_dpsi = ref_dpsi
        is_callable(ref_ddpsi)
        self.ref_ddpsi = ref_ddpsi
        
        # Gains
        is_numeric(kp_z)
        self.kp_z = np.float64(kp_z)
        is_numeric(kd_z)
        self.kd_z = np.float64(kd_z)
        is_numeric(kp_phi)
        self.kp_phi = np.float64(kp_phi)
        is_numeric(kd_phi)
        self.kd_phi = np.float64(kd_phi)
        is_numeric(kp_phi)
        self.kp_phi = np.float64(kp_phi)
        is_numeric(kd_phi)
        self.kd_phi = np.float64(kd_phi)
        is_numeric(kp_theta)
        self.kp_theta = np.float64(kp_theta)
        is_numeric(kd_theta)
        self.kd_theta = np.float64(kd_theta)
        is_numeric(kp_psi)
        self.kp_psi = np.float64(kp_psi)
        is_numeric(kd_psi)
        self.kd_psi = np.float64(kd_psi)
        
        is_numeric(g)
        self.g = np.float64(g)
        is_numeric(mass)
        self.mass = np.float64(mass)
        A = np.array(A, dtype=np.float64)
        self.Ainv = np.linalg.inv(A)
        is_numeric(jx)
        self.jx = np.float64(jx)
        is_numeric(jy)
        self.jy = np.float64(jy)
        is_numeric(jz)
        self.jz = np.float64(jz)

        self.ref_phi = ref_phi
        self.ref_theta = ref_theta
        self.ref_dphi = ref_dphi
        self.ref_dtheta = ref_dtheta
        self.ref_ddphi = ref_ddphi
        self.ref_ddtheta = ref_ddtheta

        # Configs
        is_instance(log_internals, bool)
        self.log_internals = log_internals
        self.internals = defaultdict(list)
    
    def compute(self, t: Number, xs: npt.ArrayLike, output_only:bool=False) -> Union[np.ndarray, dict]:
        # phi, dphi, theta, dtheta, psi, dpsi, x, dx, y, dy, z, dz = xs
        
        if xs.ndim==1:
            phi, dphi, theta, dtheta, psi, dpsi, _, _, _, _, z, dz = xs
        elif xs.ndim==2:
            phi, dphi, theta, dtheta, psi, dpsi, _, _, _, _, z, dz = xs.T
        else:
            raise ValueError('xs must be a vector or a matrix')
        
        ref_z = self.ref_z(t)
        ref_dz = self.ref_dz(t)
        ref_ddz = self.ref_ddz(t)
        ref_x = self.ref_x(t)
        ref_y = self.ref_y(t)
        ref_psi = self.ref_psi(t)
        ref_dpsi = self.ref_dpsi(t)
        ref_ddpsi = self.ref_dpsi(t)

        ez = ref_z - z
        dez = ref_dz - dz
        u_z = self.kp_z*ez + self.kd_z*dez + ref_ddz
        f = (u_z+self.g)*self.mass/(np.cos(phi)*np.cos(theta))

        # Roll, pitch, yaw
        # Computing remaining refs
        ref_phi = self.ref_phi(ref_x, ref_y, ref_psi, f, self.mass)
        ref_dphi = self.ref_dphi(ref_x, ref_y, ref_psi, f, self.mass)
        ref_ddphi = self.ref_ddphi(ref_x, ref_y, ref_psi, f, self.mass)
        e_phi = ref_phi-phi
        de_phi = ref_dphi-dphi

        ref_theta = self.ref_theta(ref_x, ref_y, ref_psi, ref_phi, f, self.mass)
        ref_dtheta = self.ref_dtheta(ref_x, ref_y, ref_psi, f, ref_phi, self.mass)
        ref_ddtheta = self.ref_ddtheta(ref_x, ref_y, ref_psi, f, ref_phi, self.mass)
        e_theta = ref_theta-theta
        de_theta = ref_dtheta-dtheta

        e_psi = ref_psi-psi
        de_psi = ref_dpsi-dpsi

        # Computing roll, pitch, yaw inputs
        u_phi = self.kp_phi*e_phi+ self.kd_phi*de_phi + ref_ddphi
        u_theta = self.kp_theta*e_theta+ self.kd_theta*de_theta + ref_ddtheta
        u_psi = self.kp_psi*e_psi+ self.kd_psi*de_psi + ref_ddpsi

        m_x = u_phi*self.jx
        m_y = u_theta*self.jy
        m_z = u_psi*self.jz

        f_M = np.array([f, m_x, m_y, m_z])
        fi = np.dot(self.Ainv, f_M)

        if output_only:
            return fi
        else:
            res = dict(t=t,
                    ref_z=ref_z, ref_dz=ref_dz, ref_ddz=ref_ddz, 
                    ref_x=ref_x, ref_y=ref_y, 
                    ref_psi=ref_psi, ref_dpsi=ref_dpsi, ref_ddpsi=ref_ddpsi,
                    ref_theta=ref_theta, ref_dtheta=ref_dtheta, ref_ddtheta=ref_ddtheta, 
                    ref_phi=ref_phi, ref_dphi=ref_dphi, ref_ddphi=ref_ddphi, 
                    u_z=u_z, u_phi=u_phi, u_theta=u_theta, u_psi=u_psi, 
                    f=f, m_x=m_x, m_y=m_y, m_z=m_z, 
                    f1=fi[0], f2=fi[1], f3=fi[2], f4=fi[3])
            return res


    def output(self, t: Number, xs: npt.ArrayLike) -> np.ndarray:
        if self.log_internals:
            res = self.compute(t, xs, output_only=False)
            self._log_values(**res)
            fi = np.array(res['f1'], res['f2'], res['f3'], res['f4'])
        else:
            fi = self.compute(t, xs, output_only=True)
        
        return fi
    
    def _log_values(self, **kwargs):
        for key, value in kwargs.items():
            self.internals[key].append(value)
        
    
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
        is_numeric(jx)
        self.jx = jx
        is_numeric(g)
        self.g = g
        is_numeric(mass)
        self.mass = mass
        self.A = np.array(A, dtype=np.float64)
    
    def __call__(self, t: Number, xs: npt.ArrayLike, 
        fi: npt.ArrayLike) -> np.ndarray:
        f, m_x, m_y, m_z = np.dot(self.A, fi)
        f_over_m = f/self.mass
        phi, dphi, theta, dtheta, psi, dpsi, x, dx, y, dy, z, dz = xs
        sphi = np.sin(phi)
        cphi = np.cos(phi)
        stheta = np.sin(theta)
        ctheta = np.cos(theta)
        spsi = np.sin(psi)
        cpsi = np.cos(psi)
        
        ddphi = dtheta*dpsi*((self.jy-self.jz)/self.jx) + m_x/self.jx
        ddtheta = dphi*dpsi*((self.jz-self.jx)/self.jy) + m_y/self.jy
        ddpsi = dtheta*dphi*((self.jx-self.jy)/self.jz) +  m_z/self.jz
        ddx = f_over_m*((spsi*sphi) + (cphi*stheta*cpsi))
        ddy = f_over_m*((-cpsi*sphi) + (spsi*stheta*cphi))
        ddz = -self.g+(cphi*ctheta*f_over_m)

        dxs = np.array([dphi, ddphi, dtheta, ddtheta, dpsi, ddpsi, dx, ddx, dy, ddy, dz, ddz])
        return dxs

    def output(self, t, xs):
        return np.array(xs)

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
        raise NotImplementedError
    

class ControledDrone(DynamicSystem):

    def __init__(self, controler: DroneController, drone: Drone) -> None:
        is_instance(controler, DroneController)
        self.controler=controler
        is_instance(drone, Drone)
        self.drone=drone

    def __call__(self, t, xs) -> np.ndarray:
        controler_out = self.controler.output(t, xs)
        drone_dxs = self.drone(t, xs, controler_out)
        return drone_dxs
    
    def output(self, t, xs):
        return np.array(xs)

def ref_phi(ref_x: Number, ref_y: Number, ref_psi:Number, f: Number, mass:Number) -> anp.float64:
    cpsi = anp.cos(ref_psi)
    spsi = anp.sin(ref_psi)
    ref_phi = -anp.arcsin((mass/f)*(cpsi*ref_x + spsi*ref_y))
    return ref_phi

def ref_theta(ref_x: Number, ref_y: Number, ref_psi: Number, ref_phi: Number, f: Number, mass:Number) -> anp.float64:
    cpsi = anp.cos(ref_psi)
    spsi = anp.sin(ref_psi)
    cphi = anp.cos(ref_phi)
    ref_theta = (1/cphi)*anp.arcsin((mass/f)*(-spsi*ref_x + cpsi*ref_y))
    return ref_theta


ref_dphi = egrad(ref_phi)
ref_ddphi = egrad(ref_dphi)
ref_dtheta = egrad(ref_theta)
ref_ddtheta = egrad(ref_dtheta)