clear all;
close all;
global N Phi Gamma Q1 Q2 Q0 Q12 h

% Steps
h = 1; % Sample step
h_plant = (1e-3); % Integration step


% Continuous state space
A = [0 1; 0 0];
B = [0; 1];
C = [1 0];
D = [0];

% Initial plant states
x0 = zeros(2, 1);
x0(1) = 1;
x0(2) = 0;

% Initial observer states;
x0_observer = zeros(2, 1);
x0_observer(1) = 0;
x0_observer(2) = 0;

% Discrete model of double integrator
Phi = [1 h; 0 1];    
Gamma = [h^2/2; h];
C_discrete = [1 0];
D_discrete = 0;
sysd = ss(Phi, Gamma, C, D, h);

% System order 
n = length(Phi);

% Optimization horizon
N = 20;

% Reference
reference=0;

% LQR cost matrices
Q1 = eye(n);
Q2 = 1;
Q0 = eye(n);
Q12 = 0;    

% Arbitray gains
non_optimal_poles = [0.1 0.15];
K_non_optimal = place(Phi, Gamma, non_optimal_poles);

% LQR stationary gains
[X,K,lqr_poles,info] = idare(Phi,Gamma,Q1,Q2,[],[]);
observer_eig = [0.1 0.2];

% Pertubations
noise_seed = 42;
process_noise_variance = 0.05;
measure_noise_variance = 0.05;
measure_noise_gain = 0;
wind_initial_value = 0;
wind_final_value = 0;
wind_steptime = 5;

%Kalman Filter params
G = [h^2/2; h];
Q = process_noise_variance;
R = measure_noise_variance;

% Input saturation
sat_min = -(10000);
sat_max = (10000);

% out = sim('real_time_optimal_control.slx', 30);
% 
% dirname = 'imgs';
% if not(isfolder(dirname))
%     mkdir(dirname);
% end
% 
% plot_u(out);
% plot_y(out);
% plot_j(out);
% plot_j_total(out);
% plot_x(out);
% plot_error(reference, out);