function inertia = cylinder_inertia(h,r,m)
ixx = 3*(r^2)+(h^2);
iyy = 3*(r^2)+(h^2);
izz = 6*(r^2);
inertia = [ixx 0 0;
           0 iyy 0;
           0 0 izz];
inertia = (m/12)*inertia;
end

