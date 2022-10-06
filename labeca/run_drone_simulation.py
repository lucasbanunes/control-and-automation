import numpy as np
import pandas as pd
import packages.drone as drone_models
from scipy.integrate import solve_ivp
from datetime import datetime

gravity = 10
drone_mass = 0.01
time_range = (0,10)     # Seconds
phi0 = 0
dphi0 = 0
theta0 = 0
dtheta0 = 0
psi0 = 0
dpsi0 = 0
x0 = 0
dx0 = 0
y0 = 0
dy0 = 0
z0 = 0
dz0 = 0
initial_states = [phi0, dphi0, theta0, dtheta0, psi0, dpsi0, x0, dx0, y0, dy0, z0, dz0]

d=1
ctau=1
s = np.array([0.1,0.7,0.3,0.5], dtype=np.float64)
A = np.array([
    np.ones(4),
    [0, d, 0, -d],
    [-d, 0, d, 0],
    ctau*s,
], dtype=np.float64)

controler_kwargs = dict(
    ref_x = lambda t: 1.,
    ref_y = lambda t: 1.,
    ref_z = lambda t: 1.,
    ref_dz = lambda t: 0.,
    ref_ddz = lambda t: 0.,
    ref_psi = lambda t: 0.,
    ref_dpsi = lambda t: 0.,
    ref_ddpsi = lambda t: 0.,
    kp_z = 1,
    kd_z= 1,
    kp_phi = 2,
    kd_phi = 2,
    kp_theta = 2,
    kd_theta = 2,
    kp_psi = 2,
    kd_psi = 2,
    g = gravity,
    mass = drone_mass,
    A = A,
    mx=1,
    my=1,
    mz=1,
    jx=1,
    jy=1,
    jz=1
)

drone_kwargs = dict(
    jx=1,
    jy=1,
    jz=1,
    # mx=1,
    # my=1,
    # mz=1,
    g = gravity,
    mass = drone_mass,
    A=A
)

controler = drone_models.DroneController(**controler_kwargs)
drone = drone_models.Drone(**drone_kwargs)
controled_drone = drone_models.ControledDrone(controler, drone)
res = solve_ivp(controled_drone, t_span=time_range, y0=initial_states, max_step=1e-2)

# Saving output
exec_time = datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
filename = 'drone_sim_out.csv'# f'{exec_time}_drone_sim_out.npz'
sim_out = np.concatenate((res.t.reshape(1,-1), res.y),axis=0).T
sim_out = pd.DataFrame(sim_out,columns=['t']+drone_models.states_names)
sim_out.to_csv(filename)

print('End')