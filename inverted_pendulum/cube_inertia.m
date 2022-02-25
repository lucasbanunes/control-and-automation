function inertia = cube_inertia(x,y,z,m)
ixx = (y^2)+(z^2);
iyy = (x^2)+(z^2);
izz = (x^2)+(y^2);
inertia = [ixx 0 0;
           0 iyy 0;
           0 0 izz];
inertia = (m/12)*inertia;
end

