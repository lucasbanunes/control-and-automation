%% Computing link inertias

clear;
close all;
HEIGHT = 1;
RADIUS = 2;
X = [1 0 0]';
Y = [0 1 0]';
Z = [0 0 1]';
GRAVITY = 9.8;

l = [0.187 0.34];
b = [38e-3 6.5e-3];
m = [0.25 0.15];
n_links = length(l);
inertias = double.empty(3, 3, 0);
rc = 0.5*l;

for i=1:n_links
    if i==1
        inertias(:,:,i) = cube_inertia(l(i), b(i), b(i), m(i));
    elseif i==2
        inertias(:,:,i) = cylinder_inertia(l(i), b(i), m(i));
    end
end

%% Comptuing M matrix

theta = sym('theta', [2 1], 'real');
sym_inertia = sym('sym_inertia', [3 3]);
R01 = rotz(theta(1));
R12 = rotx(theta(2));

M1 = [inertias(3,3,1) + (m(1)*(l(1)^2)/4), 0;
      0                               , 0];

M2A = (m(2)*(l(1)^2)) + ((Z'*R12)*(inertias(:, :, 2))*(R12'*Z)) + (m(2)*(l(2)^2)*(sin(theta(2))^2));
M2C = -m(2)*l(2)*l(1)*cos(theta(2))/2;
M2B=M2C;
M2D = inertias(1,1,2) + (m(2)*(l(2)^2)/4);
M2 = [M2A, M2B; M2C M2D];

M = M1 + M2;
get_M = matlabFunction(M);
inv_M = inv(M);
get_inv_M = matlabFunction(inv_M);

%% Computing C matrix
theta_dot = sym('theta_dot', [2 1], 'real');
MD = sym(zeros(n_links, n_links));
for i=1:n_links
    MD(:, i) = diff(M, theta(i))*theta_dot;
end
C = MD - (0.5*MD');
get_C = matlabFunction(C);

%% Computing G matrix
G = [0; -0.5*m(2)*GRAVITY*l(2)*sin(theta(2))];
get_G = matlabFunction(G);

%% Computing motor joint dynamics
% Motor parameters
km = 0.00767;
ra = 2.6;
la = 0.18e-3;
joint_inertias = 2.8e-6+2.27e-5+5e-7;
armature_inertia = 2.86e-7;
kr = 70;
jt = joint_inertias + (kr^2)*armature_inertia;

Mm = [jt 0; 0 0];
M_hat = M+Mm;
get_M_hat = matlabFunction(M_hat);
inv_M_hat = inv(M_hat);
get_inv_M_hat = matlabFunction(inv_M_hat);

Cm = [km*km*kr*kr/ra 0;0 0];
Vm = [kr*km/ra 0; 0 0];

%% Comparing simulation with experimental data
exp_data = load('exp_data.mat');

theta0 = zeros(n_links, 1);
theta_dot0 = zeros(n_links, 1);
out = sim('simulation.slx', 'StartTime', '0', 'StopTime', '20');

figure;
grid;
hold on;
plot(out.mani_theta, 'LineWidth', 2.0);
hold on;
plot(exp_data.data01(:, 1), deg2rad(exp_data.data01(:, 2)), 'LineWidth', 2.0)
hold on;
plot(exp_data.data01(:, 1), deg2rad(exp_data.data01(:, 3)), 'LineWidth', 2.0);
xlabel('Time(seconds)');
ylabel('rad');
title('Estado inicial no equilíbrio instável (\theta=0)');
saveas(gcf, 'mani_theta_0.png');
hold off;

theta0(2) = pi/2;
out = sim('simulation.slx', 'StartTime', '0', 'StopTime', '20');

figure;
grid;
hold on;
plot(out.mani_theta, 'LineWidth', 2.0);
hold on;
plot(exp_data.data02(:, 1), deg2rad(exp_data.data02(:, 2)), 'LineWidth', 2.0)
hold on;
plot(exp_data.data02(:, 1), deg2rad(exp_data.data02(:, 3)), 'LineWidth', 2.0);
xlabel('Time(seconds)');
ylabel('rad');
title('Estado inicial com \theta_2 = 90 graus');
saveas(gcf, 'mani_theta_90.png');
hold off;

figure;
grid;
hold on;
plot(out.motor_theta, 'LineWidth', 2.0);
hold on;
plot(exp_data.data03(:, 1), deg2rad(exp_data.data03(:, 2)), 'LineWidth', 2.0)
hold on;
plot(exp_data.data03(:, 1), deg2rad(exp_data.data03(:, 3)), 'LineWidth', 2.0);
xlabel('Time(seconds)');
ylabel('rad');
title('Sistema com motor acoplado no estado inicial com \theta_2 = 90 graus');
saveas(gcf, 'motor_theta_90.png');
hold off;
