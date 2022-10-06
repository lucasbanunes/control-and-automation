import numpy as np
import numpy.typing as npt
from packages.models import DynamicSystem
from packages.utils import is_callable, is_numeric, is_instance
from typing import Callable
from numbers import Number
from autograd import grad
import autograd.numpy as anp

states_names = ['phi','dphi','theta','dtheta','psi','dpsi','x','dx','y','dy','z','dz']

class DroneController():

    def __init__(self, ref_x: Callable[[Number],Number], ref_y: Callable[[Number],Number], 
        ref_z: Callable[[Number],Number], ref_dz: Callable[[Number],Number], ref_ddz: Callable[[Number],Number], 
        ref_psi: Callable[[Number],Number], ref_dpsi: Callable[[Number],Number], ref_ddpsi: Callable[[Number],Number], 
        kp_z: Number, kd_z: Number, kp_phi:Number, kd_phi:Number, kp_theta:Number, kd_theta:Number, kp_psi:Number, kd_psi:Number,
        A: npt.ArrayLike, mx: Number, my: Number, mz: Number, jx: Number, jy: Number, jz: Number,
        g: Number, mass: Number):
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
        kp_z: Number
            Proportional gain for z control
        kd_z: Number
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
        is_numeric(mx)
        self.mx = np.float64(mx)
        is_numeric(my)
        self.my = np.float64(my)
        is_numeric(mz)
        self.mz = np.float64(mz)
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
    
    def __call__(self, t: Number, xs: npt.ArrayLike) -> np.ndarray:

        # phi, dphi, theta, dtheta, psi, dpsi, x, dx, y, dy, z, dz = xs
        phi, dphi, theta, dtheta, psi, dpsi, _, _, _, _, z, dz = xs
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
        uz = self.kp_z*ez + self.kd_z*dez + ref_ddz
        f = (uz+self.g)*self.mass/(np.cos(phi)*np.cos(theta))

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

        mx = u_phi*self.jx
        my = u_theta*self.jy
        mz = u_psi*self.jz
        
        fi = np.dot(self.Ainv, np.array((f, mx, my, mz)))
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
        pass
    

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


ref_dphi = grad(ref_phi)
ref_ddphi = grad(ref_dphi)
ref_dtheta = grad(ref_theta)
ref_ddtheta = grad(ref_dtheta)