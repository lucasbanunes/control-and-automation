function dstates = getStates(t,states,g,m,jx,jy,jz, kpz, kdz)
    %% Initalizing system

    phi = states(1);
    dphi = states(2);
    theta = states(3);
    dtheta = states(4);
    psi = states(5);
    dpsi = states(6);
    x = states(7);
    dx = states(8);
    y = states(9);
    dy = states(10);
    z = states(11);
    dz = states(12);

    spsi = sin(psi);
    cpsi = cos(psi);
    sphi = sin(phi);
    cphi = cos(phi);
    stheta = sin(theta);
    ctheta = cos(theta);
    
    %% References
    xd = (0*(2*sin(3*t))+1)';
    yd = (0*(2*sin(3*t))+1)';
    zd = (0*(2*sin(3*t))+1)';
    dzd=0;
    ddzd=0;
    psid = (0*(2*sin(3*t))+pi/4)';
    
    %% Control
    e = zd-z;
    de = dzd-dz;
    uz = kpz*e + kdz*de+ddzd;
    f = (uz+g)*m/(cphi*ctheta);

    mx=1;
    my=1;
    mz=1;

    %% System
    f_over_m = f/m;

    ddphi = dtheta*dpsi*((jy-jz)/jx) + mx/jx;
    ddtheta = dphi*dpsi*((jz-jx)/jy) + my/jy;
    ddpsi = dtheta*dphi*((jx-jy)/jz) +  mz/jz;
    ddx = f_over_m*((spsi*sphi) + (cphi*stheta*cpsi));
    ddy = f_over_m*((-cpsi*sphi) + (spsi*stheta*cphi));
    ddz = -g+(cphi*ctheta*f_over_m);

    dstates= [dphi;ddphi;dtheta;ddtheta;dpsi;ddpsi;dx;ddx;dy;ddy;dz;ddz];
end