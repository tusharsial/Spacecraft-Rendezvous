function dstate = Rendezvous(t,state)

x = state(1);
y = state(2);
z = state(3);


%%% Kinematics
%vel = state(4:6);

%%%Gravity Model
Reference_planet
w = 0.000592034043275745;

%%% Dynamics 
%x_ddot = 2*w*state(5) + w*w*(R0+x) - myu*(R0+x)/(norm([R0+x; y; z])^(3/2));
%y_ddot = -2*w*state(4) + w*w*y - myu*y/(norm([(R0+x); y; z])^(3/2));
%z_ddot = -myu*z/(norm([(R0+x); y; z])^(3/2));

%x_ddot = 2*w*state(5) + 3*w*w*x;
%y_ddot = -2*w*state(4);
%z_ddot = -w*w*z;
%accel = [x_ddot; y_ddot; z_ddot];

A = [0 0 0 1 0 0;0 0 0 0 1 0; 0 0 0 0 0 1; 3*w*w 0 0 0 2*w 0; 0 0 0 -2*w 0 0; 0 0 -w*w 0 0 0];
B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
K = [0.001 0 0 0.3 0 0; 0 0.01 0 0 7 0; 0 0 0.01 0 0 7];
%Q = eye(6);
%R = eye(3);
%K = lqr(A,B,Q,R);
%U = -K*state;
U = [0;0;0];
Xdot = A*state + B*U;

%%% Return derivatives vector
%dstate = [vel; accel];
dstate = Xdot;