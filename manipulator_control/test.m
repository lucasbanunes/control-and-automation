clear;
omega_n = 5;
ksi = 1;
sys_order = 3;
n_links = 2;
M_bar = [sym(110001/1000) sym(113/5); sym(113/5) sym(65/2)];
syms ka1 ka2 kp1 kp2 ki1 ki2 kd1 kd2 real;
Ka = diag([ka1, ka2]);
Kd = diag([kd1, kd2]);
Kp = diag([kp1, kp2]);
Ki = diag([ki1, ki2]);
%Ki = sym('Ki', [n_links, n_links], 'real');
A = Ka-M_bar;
B = Kd;
C = Ki + (Kd*Kp);
D = Ki*Kp;
syms s;
eq_matrix = A*s^3 + B*s^2 + C*s + D;
% err_theta = sym('err_theta', [2,1]);
% eq = eq_matrix*err_theta;
alphar = 0.1;
beta = acos(ksi);
tr = (pi-beta)/omega_n;
pr = -log(alphar)/tr;
pl = 10;
desired_poly = conv([1 pl], [1 12 36]);
syms k real;
eqs = [eig(A) == desired_poly(1)*ones(n_links, 1),
       eig(B) == desired_poly(2)*ones(n_links, 1),
       eig(C) == desired_poly(3)*ones(n_links, 1),
       eig(D) == desired_poly(4)*ones(n_links, 1)];
% a = (2*ksi*omega_n*pl - (pl^2))/((p^2) + (omega_n^2) - (2*omega_n*pl));
% b = ((4*(ksi^2)*(omega_n^2)*pl) - (2*omega_n*(pl^2)) - ((omega_n^2)*pl))/((p^2) + (omega_n^2) - (2*ksi*omega_n*pl));