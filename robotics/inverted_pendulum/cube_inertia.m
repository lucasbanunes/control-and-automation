function inertia = cube_inertia(l,h,w,m)
ixx = (h^2)+(w^2);
iyy = (l^2)+(w^2);
izz = (l^2)+(h^2);
inertia = [ixx 0 0;
           0 iyy 0;
           0 0 izz];
inertia = (m/12)*inertia;
end

