%%% Simulation of Relative motion (Satellite Rendezvous)
disp('Relative Motion Simulation started')

%%% Initial conditions
w = sqrt(myu/semi_major_tar^3);
%w = sqrt(myu/r_tar^3);

%%% Extracting states of chaser satellite in rotating coordinate frame  
x0 = r_ct_R(1);
y0 = r_ct_R(2);
z0 = r_ct_R(3);

xdot0 = v_ct_R(1); % in m/s
ydot0 = v_ct_R(2); % in m/s
zdot0 = v_ct_R(3); % in m/s

stateinitial = [x0; y0; z0; xdot0; ydot0; zdot0];

%%% Implementing Modified CW equations

rho = 1+e*cos(theta);
s = rho*sin(theta);
c = rho*cos(theta);
s_d = cos(theta)+e*cos(2*theta);
c_d = -(sin(theta)+e*sin(2*theta));
k = sqrt(h/p^2);
J = k^2*(t - t0);

pseudo = (1/1-e^2)*[1-e^2 3*e*s(1/rho + 1/rho^2) -e*s*(1+1/rho) -e*c+2; 0 -3*s*(1/rho + e^2/rho^2) s*(1+1/rho) c-2*e; 0 -3*(c/rho + e) c*(1+1/rho)+e -s; 0 3*rho+e^2-1 -rho^2 e*s];
stateout_inplane = [1 -c*(1+1/rho) s*(1+1/rho) 3*rho^2*J; 0 s c (2-3*e*s*J); 0 2*s 2*c-e 3*(1-2*e*s*J); 0 s_d c_d -3*e(s_d*J + s/rho^2)]*pseudo;