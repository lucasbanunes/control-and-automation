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
rc = double.empty(0, 1);

for i=1:n_links
    if i==1
        inertias(:,:,i) = cube_inertia(l(i), b(i), b(i), m(i));
    elseif i==2
        inertias(:,:,i) = cylinder_inertia(l(i), b(i), m(i));
    end
    rc(i) = l(i)/2;
end

%% Comptuing M matrix

theta = sym('theta', [2 1], 'real');
sym_inertia = sym('inertia', [3 3], 'real');          
R01 = rotz(theta(1));
R12 = roty(pi/2)*rotz(theta(2));

x2_cross_z1 = skew(R12*[1 0 0]')*[0 0 1]';
x2_cross_y1 = skew(R12*[1 0 0]')*[0 1 0]';
z1I2z1 = (((R12')*[0 0 1]')')*sym_inertia*((R12')*[0 0 1]');

M1 = [inertias(3,3,1) + m(1)*(rc(1)^2) 0;
      0                                              0];

M2A = m(2)*(l(1)^2) + (R12'*Z)'*inertias(:, :, 2)*(R12'*Z) + m(2)*(rc(2)^2);
M2B = -m(2)*l(1)*rc(2)*dot(Y, R12*Y) + (R12'*Z)'*inertias(:, :, 2)*Z + m(2)*(rc(2)^2);
M2C = -m(2)*rc(2)*cos(theta(2))*dot(R12*Z, X) + Z'*inertias(:, :, 2)*(R12'*Z) + m(2)*(rc(2)^2);
M2D = inertias(3,3,2) + m(2)*(rc(2)^2);
M2 = [M2A, M2B; M2C M2D];

M = M1 + M2;
M = simplify(M, 400);

%% Computing C matrix
theta_dot = sym('theta_dot', [2 1], 'real');
MD = [diff(M, theta(1))*theta_dot, diff(M, theta(2))*theta_dot];
C = (MD-(0.5*(MD')));

%% Computing G matrix
g = m(1)*dot(-GRAVITY*Z, R01*rc(1)*X) + m(2)*dot(-GRAVITY*Z, R01*(l(1)*X + R12*rc(2)*-X));
G = [diff(g, theta(1)), diff(g, theta(2))]';