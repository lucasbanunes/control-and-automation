%% Manipulator model
clear;
close all;

% Manipulator parameters
GRAVITY = 9.98;
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
c = [cos(theta(1)) cos(theta(2))];
c12 = cos(theta(1)+theta(2));
s = [sin(theta(1)) sin(theta(2))];

M = sym(zeros(n_links, n_links));

M(1,1) = il(1) + (ml(1)*(l(1)^2)) + ((kr(1)^2)*im(1)) + il(2);
M(1,1) =  M(1,1) + (ml(2)*((a(1)^2) + (l(2)^2) + 2*a(1)*l(2)*c(2))) + im(2) + (mm(2)*(a(1)^2));
M(1,2) = il(2) + (ml(2)*((l(2)^2)+(a(1)*l(2)*c(2)))) + (kr(2)*im(2));
M(2,1) = M(1,2);
M(2,2) = il(2) + (ml(2)*(l(2)^2)) + ((kr(2)^2)*im(2));
get_M = matlabFunction(M);
inv_M = inv(M);
get_inv_M = matlabFunction(inv_M);

h = -ml(2)*a(1)*l(2)*s(2);
C = sym(zeros(n_links, n_links));
C(1,1) = h*theta_dot(2);
C(1,2) = h*(theta_dot(1)+theta_dot(2));
C(2,1) = -h*theta_dot(1);
get_C = matlabFunction(C);

G = sym(zeros(n_links, 1));
G(1) = ((ml(1)*l(1)) + (mm(2)*a(1)) + (ml(2)*a(1)))*GRAVITY*c(1) + (ml(2)*l(2)*GRAVITY*c12);
G(2) = ml(2)*l(2)*GRAVITY*c12;
get_G = matlabFunction(G);

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
M_bar = [110002/1000 113/5; 113/5 62/5];
Kd = first_order*M_bar;
Ki = zero_order*M_bar;
Gm = tf([first_order zero_order], [1 first_order zero_order 0], 'Name', 'Gm');
rlocus(Gm);
Kp = diag([5.67 5.67]);  %Computed by the previous rlocus
Ka = M_bar;
ctrl = 'P-PI';
% for test=1:n_tests
%     dirname = sprintf('test%i', test);
%     if not(isfolder(dirname))
%         mkdir(dirname);
%     end
%     omega = omegas(:,test);   %Needed for simulink simulation
%     if test == 3
%         stoptime=sim_time/2;
%     else
%         stoptime=sim_time;
%     end
%     out = sim('ppi.slx', 'StartTime', '0', 'StopTime', num2str(stoptime));
%     plot_theta(out, dirname, ctrl);
%     plot_theta_dot(out, dirname, ctrl);
%     plot_theta_ddot(out, dirname, ctrl);
% end

%% Computed torque
% ctrl = 'Torque computado';
% computed_kp = omega_n^2;
% computed_kd = omega_n*ksi*2;
% for test=1:n_tests
%     dirname = sprintf('test%i', test);
%     if not(isfolder(dirname))
%         mkdir(dirname);
%     end
%     omega = omegas(:,test);   %Needed for simulink simulation
%     if test == 3
%         stoptime=sim_time/2;
%     else
%         stoptime=sim_time;
%     end
%     out = sim('computed_torque.slx', 'StartTime', '0', 'StopTime', num2str(stoptime));
%     plot_theta(out, dirname, ctrl);
%     plot_theta_dot(out, dirname, ctrl);
%     plot_theta_ddot(out, dirname, ctrl);
% end
