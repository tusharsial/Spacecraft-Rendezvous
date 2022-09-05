function [dstate,U] = Rendezvous_with_Perturbation(t,state,w,R0)

% This function implements the CW equations and outputs state vector in
% LVLH frame. J2 perturbations have also been included.

i = 0*pi/180;
x = state(1);
y = state(2);
z = state(3);


%%% Kinematics
vel = state(4:6);

%%%Gravity Model
Reference_planet

%%% Dynamics (
%x_ddot = 2*w*state(5) + w*w*(R0+x) - myu*(R0+x)/(norm([R0+x; y; z])^(3/2));
%y_ddot = -2*w*state(4) + w*w*y - myu*y/(norm([(R0+x); y; z])^(3/2));
%z_ddot = -myu*z/(norm([(R0+x); y; z])^(3/2));

s = (3*J2*Re*Re/(8*R0*R0))*(1+3*cos(2*i));
c = sqrt(1+s);

%%% Dynamics with Controller 

% LQR Controller has been implemented.

A = [0 0 0 1 0 0;0 0 0 0 1 0; 0 0 0 0 0 1; (5*c-2)*w*w 0 0 0 2*w*c 0; 0 0 0 -2*w*c 0 0; 0 0 -(3*c-2)*w*w 0 0 0];
B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
%K = [0.001 0 0 0.3 0 0; 0 0.01 0 0 7 0; 0 0 0.01 0 0 7];

Q = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;0 0 0 100 0 0; 0 0 0 0 100 0; 0 0 0 0 0 100;];
%Q = 10*eye(6);
R = 10^9*[100 0 0; 0 100 0; 0 0 100];
%R = [100 0 0; 0 10 0; 0 0 10^2];
K = lqr(A,B,Q,R);
U = -K*state;
%U = [0;0;0];

Xdot = A*state + B*U;
%disp(U);

%%% Return derivatives vector
%accel = [x_ddot;y_ddot;z_ddot];
%dstate = [vel; accel];
dstate = Xdot;