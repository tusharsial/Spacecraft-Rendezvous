function dstatedt = Derivatives(state,U,R0,w)

i = 0*pi/180;

%%%Gravity Model
Reference_planet

s = (3*J2*Re*Re/(8*R0*R0))*(1+3*cos(2*i));
c = sqrt(1+s);

%%% Dynamics with Controller 

% A simple statefeedback controller has been used for now. This controller
% will be upgraded to include LQR.

A = [0 0 0 1 0 0;0 0 0 0 1 0; 0 0 0 0 0 1; (5*c-2)*w*w 0 0 0 2*w*c 0; 0 0 0 -2*w*c 0 0; 0 0 -(3*c-2)*w*w 0 0 0];
B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];

%K = [0.001 0 0 0.3 0 0; 0 0.01 0 0 7 0; 0 0 0.01 0 0 7];
%U = -K*state;

Xdot = A*state + B*U;

%%% Return derivatives vector
dstatedt = Xdot;