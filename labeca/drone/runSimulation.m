initParams;
fprintf("Starting simulation\n");
[T, STATES] = ode45(@(t,x) getStates(t,x,g,m,jx,jy,jz,kpz,kdz), [0 1], initial_states);
fprintf("Finished simulation\n");