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
R01 = rotz(theta(1));
R12 = roty(pi/2)*rotz(theta(2));

x2_cross_z1 = skew(R12*X)*Z;
x2_cross_y1 = skew(R12*X)*Y;
z1_dot_x2 = dot(R12'*Z, X);

M1 = [inertias(3,3,1) + m(1)*(rc(1)^2), 0;
      0                               , 0];

M2A = m(2)*(l(1)^2) + (Z'*R12)*(inertias(:, :, 2))*(R12'*Z) + m(2)*(rc(2)^2)*(1-(cos(theta(2)^2)));
M2C = -m(2)*rc(2)*l(1)*cos(theta(2));
M2B=M2C;
M2D = inertias(3,3,2) + m(2)*(rc(2)^2);
M2 = [M2A, M2B; M2C M2D];

M = M1 + M2;
M = simplify(M, 400);

%% Computing C matrix
theta_dot = sym('theta_dot', [2 1], 'real');
MD = [diff(M, theta(1))*theta_dot, diff(M, theta(2))*theta_dot];
C = MD - (0.5*MD');
C = simplify(C, 100);
get_C = matlabFunction(C);

%% Computing G matrix
g = -1*(m(1)*dot(-GRAVITY*Z, R01*rc(1)*X) + m(2)*dot(-GRAVITY*Z, R01*(l(1)*X + R12*(rc(2)*-X))));
G = [diff(g, theta(1)), diff(g, theta(2))]';
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

%Model for motor with circuit dynamics
Am = [(kr*la*jt)/ra 0; 0 0];
inv_Am = pinv(Am);
Cm = [(km^2*kr/ra) 0; 0 0];
Vm = [km*kr/ra 0; 0 0];

%Model for motor without circuit dynamics
alpham = ra*jt/(km^2);
Cm_hat = [kr*jt/alpham 0; 0 0];
Vm_hat = [(kr*jt)/(alpham*km) 0; 0 0];

Mm = [(joint_inertias + (kr^2)*armature_inertia) 0; 0 0];
M = M+Mm;
get_M = matlabFunction(M);
det_M = simplify(det(M), 100);
inv_M = (1/det_M)*[M(2,2) -M(1,2);
                  -M(2,1)  M(1,1)];
get_inv_M = matlabFunction(inv_M);
Cm_hat = [km*kr 0;0 0];

%% Simulating
theta0 = zeros(n_links, 1);
theta0(2) = pi/2;
theta_dot0 = zeros(n_links, 1);
theta_ddot0 = zeros(n_links, 1);

out = sim('simulation.slx', 'StartTime', '0', 'StopTime', '20' );

exp_data = load('exp_data.mat');
figure;
grid;
hold on;
plot(out.approx_theta);
hold on;
plot(exp_data.data02(:, 1), deg2rad(exp_data.data02(:, 2)), exp_data.data02(:, 1), deg2rad(exp_data.data02(:, 3)));
legend('\theta_1^{sim}', '\theta_2^{sim}', '\theta_1^{exp}', '\theta_2^{exp}');
xlabel('Time(seconds)');
ylabel('rad');