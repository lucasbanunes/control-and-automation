%% Manipulator model
clear;
close all;

% Nominal manipulator parameters
g = 9.98;
n_links = 2;
a = ones(n_links, 1);
l = a/2;
ml = 50*ones(2,1);
il = 10*ones(n_links,1);
kr = 100*ones(n_links,1);
mm = 5*ones(n_links,1);
im = 0.001*ones(n_links,1);

theta = sym('theta', [n_links,1]);
theta_dot = sym('theta_dot', [n_links, 1]);

[nominal_M,nominal_C,nominal_G] = get_manipulator(g,a,l,ml,il,kr,mm,im,theta,theta_dot);
get_nominal_M = matlabFunction(nominal_M);
inv_nominal_M = inv(nominal_M);
get_inv_nominal_M = matlabFunction(inv_nominal_M);
get_nominal_C = matlabFunction(nominal_C);
get_nominal_G = matlabFunction(nominal_G);

real_ml = [50; 50];
[real_M,real_C,real_G] = get_manipulator(g,a,l,real_ml,il,kr,mm,im,theta,theta_dot);
get_real_M = matlabFunction(real_M);
inv_real_M = inv(real_M);
get_inv_real_M = matlabFunction(inv_real_M);
get_real_C = matlabFunction(real_C);
get_real_G = matlabFunction(real_G);

%% Control params
theta0 = zeros(n_links, 1);
theta_dot0 = zeros(n_links,1);
omega_n = 5;
ksi = 1;
omega1 = [pi; 1.2*pi];
omegas = [omega1 0.5*omega1 3*omega1];
dc = [-0.75;3.75];
n_tests = size(omegas);
n_tests = n_tests(2);
sim_time=20;

%% P-PI control
desired_poles = [-10 -50];
first_order = abs(sum(desired_poles));
zero_order = abs(prod(desired_poles));
M_bar = [110.001 0; 0 32.5];
Kd = first_order*M_bar;
Ki = zero_order*M_bar;
Gm = tf([first_order zero_order], [1 first_order zero_order 0], 'Name', 'Gm');
rlocus(Gm);
Kp = diag([5.67 5.67]);  %Computed by the previous rlocus
Ka = M_bar;
ctrl = 'P-PI';
for test=1:n_tests
    dirname = sprintf('test%i', test);
    if not(isfolder(dirname))
        mkdir(dirname);
    end
    omega = omegas(:,test);   %Needed for simulink simulation
    if test == 3
        stoptime=sim_time/2;
    else
        stoptime=sim_time;
    end
    out = sim('ppi.slx', 'StartTime', '0', 'StopTime', num2str(stoptime), 'FixedStep', '0.01');
    plot_theta(out, dirname, ctrl);
    plot_theta_dot(out, dirname, ctrl);
    plot_theta_ddot(out, dirname, ctrl);
    plot_error(out, dirname, ctrl);
end

%% Computed torque
ctrl = 'Torque computado';
computed_kp = omega_n^2;
computed_kd = omega_n*ksi*2;
for test=1:n_tests
    dirname = sprintf('test%i', test);
    if not(isfolder(dirname))
        mkdir(dirname);
    end
    omega = omegas(:,test);   %Needed for simulink simulation
    if test == 3
        stoptime=sim_time/2;
    else
        stoptime=sim_time;
    end
    out = sim('computed_torque.slx', 'StartTime', '0', 'StopTime', num2str(stoptime));
    plot_theta(out, dirname, ctrl);
    plot_theta_dot(out, dirname, ctrl);
    plot_theta_ddot(out, dirname, ctrl);
    plot_error(out, dirname, ctrl);
end
