clear;

%% Initializng params
e3 = [0 0 1]';
g = 10;
d=1;
ctau=max(d)*2;
s = ones(1,4);

Mc = [ones(1,4); 
      0 d 0 -d;
      -d 0 d 0;
      ctau*s];
m = 1; %Massa da estrutura
jx=1; %Inercia da estrutura em x
jy=1; % Inercia da estrutura em y
jz=1; % Structure inertia at z axis
%% Control params
kpz=1;
kdz=1;

%% Initial states
phi0 = 0;
dphi0 = 0;
theta0 = 0;
dtheta0 = 0;
psi0 = 0;
dpsi0 = 0;
x0 = 0;
dx0 = 0;
y0 = 0;
dy0 = 0;
z0 = 0;
dz0 = 0;
initial_states = [phi0 dphi0 theta0 dtheta0 psi0 dpsi0 x0 dx0 y0 dy0 z0 dz0]';