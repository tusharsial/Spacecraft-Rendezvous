function [C, Cdot] = ECI2LVLH(state_out)

r = state_out(1:3); % position vector in ECI frame
rnorm = norm(r); % magnitude of position vector
r_hat = r/rnorm; % unit position vector

v = state_out(4:6); % velocity vector in ECI frame
vnorm = norm(v); % magnitude of velocity vector
v_hat = v/vnorm; % unit velocity vector

h = cross(r,v); % angular momentum vector in ECI frame
hnorm = norm(h); 
h_hat = h/hnorm;

ex_hat = r_hat;
ez_hat = h_hat;
ey_hat = cross(ez_hat,ex_hat);

ex_dot = uhat_d(r,v);
ey_dot = cross(ez_hat,ex_dot);
ez_dot = 0;

C(1,:) = ex_hat;
C(2,:) = ey_hat;
C(3,:) = ez_hat;

Cdot(1,:) = ex_dot;
Cdot(2,:) = ey_dot;
Cdot(3,:) = ez_dot;
