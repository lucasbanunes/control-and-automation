function [M, C, G] = get_manipulator(g, a, l, ml, il, kr, mm, im, theta, theta_dot)
c = [cos(theta(1)) cos(theta(2))];
s = [sin(theta(1)) sin(theta(2))];
c12 = cos(theta(1)+theta(2));
M(1,1) = il(1) + (ml(1)*(l(1)^2)) + ((kr(1)^2)*im(1)) + il(2) + (ml(2)*((a(1)^2) + (l(2)^2) + 2*a(1)*l(2)*c(2))) + im(2) + (mm(2)*(a(1)^2));
M(1,2) = il(2) + (ml(2)*((l(2)^2)+(a(1)*l(2)*c(2)))) + (kr(2)*im(2));
M(2,1) = M(1,2);
M(2,2) = il(2) + (ml(2)*(l(2)^2)) + ((kr(2)^2)*im(2));

h = -ml(2)*a(1)*l(2)*s(2);
C(1,1) = h*theta_dot(2);
C(1,2) = h*(theta_dot(1)+theta_dot(2));
C(2,1) = -h*theta_dot(1);
C(2,2) = 0;

G(1) = ((ml(1)*l(1)) + (mm(2)*a(1)) + (ml(2)*a(1)))*g*c(1) + (ml(2)*l(2)*g*c12);
G(2) = ml(2)*l(2)*g*c12;
end

