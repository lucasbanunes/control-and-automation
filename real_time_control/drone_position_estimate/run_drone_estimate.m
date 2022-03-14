clear all;
close all;
% global N Phi Gamma Q1 Q2 Q0 Q12 h

% Steps
h = 1; % Sample step
h_plant = (1e-3); % Integration step

% Noise parameters
cov_an = 1*eye(3);
cov_aw = 1*eye(3);
cov_gps = .01*eye(3);
Q = [cov_an zeros(3);
     zeros(3) cov_aw]; % process_noise_covariance
R = cov_gps;   % measure_noise_variance

%Error system
Rm = eye(3);
F=[zeros(3) eye(3)   zeros(3);
     zeros(3) zeros(3) -Rm      ;
     zeros(3) zeros(3) zeros(3)];
H=[eye(3) zeros(3) zeros(3)];
B_error = zeros(9,6);
D_error = zeros(3,6);
G = [zeros(3) zeros(3);
     -Rm        zeros(3);
     zeros(3) eye(3)];
n = length(F);

% Computing exponencial for discrete error system
% F^3 = 0 so we compute until F^2
F_discrete = eye(n);
aux = eye(n);
for i=1:2
    aux = (F*h)^i;
    F_discrete = F_discrete + (aux/factorial(i));
end
B_discrete = zeros(9,6);
D_discrete = zeros(3,6);
H_discrete = H;
G_discrete = eye(9);
Q_discrete = G*Q*G'*h;
R_discrete = R/h;

% Observer parameters;
x0_observer = zeros(n, 1);
% x0_observer(1:3, 1) = 0.1;
% L = lqed(F, G, H, Q, R, h);
% L = dlqe(F_discrete, G_discrete, H_discrete, Q_discrete, R_discrete);
L = lqe(F, G, H, Q, R);

% Drone parameters
p0=zeros(3,1);
v0=zeros(3,1);

% Reference
at = [1;0;-9.98];
g = [0;0;-9.98];

% Simulating
simulation = 'drone_position_estimate_continuous.slx';
out = sim(sprintf('simulations/%s', simulation), 'FixedStep', num2str(h_plant), 'StartTime', '0', 'StopTime', '100');

save_file=true;
dirname = sprintf('imgs/%s', strrep(simulation, '.slx', ''));
if not(isfolder(dirname)) && save_file
    mkdir(dirname);
end

plot_xt(out, save_file, dirname);
plot_deltax(out, save_file, dirname);
plot_real_x(out, save_file, dirname);
plot_real_error(out, save_file, dirname);
plot_am(out, save_file, dirname);