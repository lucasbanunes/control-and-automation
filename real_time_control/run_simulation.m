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
G = 0*eye(n);
Q = 0*eye(n);   % process_noise_variance
R = 0;   % measure_noise_variance

% Initial plant states
x0 = zeros(2, 1);
x0(1) = 1;
x0(2) = 0;

% Discrete model of double integrator
Phi = [1 h; 0 1];    
Gamma = [h^2/2; h];
C_discrete = [1 0];
D_discrete = 0;
G_discrete = 0*eye(n);
Q_discrete = G*Q*(G')*h;   %For h sufficiently small
R_discrete = R*h;
sysd = ss(Phi, Gamma, C_discrete, D_discrete, h);

% Observer parameters;
% L = lqe(Phi, G_discrete, C_discrete, Q_discrete, R_discrete); %For Kalman
% L = 100*ones(n,1);
L = place(Phi', C_discrete', [0.01;0.03]);
x0_observer = zeros(2, 1);
x0_observer(1) = 0;
x0_observer(2) = 0;

% Optimization horizon
N = 20;

% Reference
reference=0;

% LQR cost matrices
Q1 = eye(n);
Q2 = 1;
Q0 = eye(n);
Q12 = 0;

%Control Gain computed for static LQR
% [X,K,lqr_poles,info] = idare(Phi,Gamma,Q1,Q2,[],[]);
[K, S, CLP] = lqr(sysd, Q1, Q2);

% Input saturation
sat_min = -(10000);
sat_max = (10000);

% Pertubations
noise_seed = 42;
wind_initial_value = 0;
wind_final_value = 0;
wind_steptime = 5;

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