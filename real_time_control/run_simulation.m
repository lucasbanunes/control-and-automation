clear all;
close all;
global N Phi Gamma Q1 Q2 Q0 Q12 h

% Steps
h = 1; % Sample step
h_plant = (1e-3); % Integration step

% Continuous plant state space (double integrator)
A = [0 1; 0 0];
n = length(A);    % System order
B = [0; 1];
C = [1 0];
D = 0;
G = eye(n);% eye(n);
Q = 0.01*eye(n);   % process_noise_covariance
R = 0.01;   % measure_noise_variance

% Initial plant states
x0 = zeros(2, 1);
x0(1) = 1;
x0(2) = 0;

% Discrete model of double integrator
Phi = [1 h; 0 1];    
Gamma = [h^2/2; h];
C_discrete = [1 0];
D_discrete = 0;
G_discrete = eye(n);
Q_discrete = G*Q*(G')*h;   %For h sufficiently small
R_discrete = R*h;
sysd = ss(Phi, Gamma, C_discrete, D_discrete, h);

% LQR cost matrices
Q1 = eye(n);
Q2 = 1;
Q0 = eye(n);
Q12 = 0;

%Control Gain computed for static LQR
% [X,K,lqr_poles,info] = idare(Phi,Gamma,Q1,Q2,[],[]);
[K, S, CLP] = lqr(sysd, Q1, Q2);

% Observer parameters;
x0_observer = zeros(2, 1);
% observer_eig = [0.03; 0.01];
L = lqe(A, G_discrete, C_discrete, Q_discrete, R_discrete);

% Optimization horizon
N = 20;

% Reference
reference=0;

% Input saturation
sat_min = -(10000);
sat_max = (10000);

% Pertubations
noise_seed = 42;
wind_initial_value = 0;
wind_final_value = 0;
wind_steptime = 5;

out = sim('simulations/lqr_control_kalman_observer.slx', 'FixedStep', num2str(h_plant));

save_file=false;
dirname = 'imgs';
if not(isfolder(dirname)) && save_file
    mkdir(dirname);
end

plot_u(out, save_file);
plot_y(out, save_file);
plot_cost(out, save_file);
plot_total_cost(out, save_file);
plot_x(out, save_file);
plot_xhat(out, save_file);
plot_error(reference, out, save_file);