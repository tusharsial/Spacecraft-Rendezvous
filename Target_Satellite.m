function dstatedt = Target_Satellite(t,state)
%{ 
This function returns the velocity and acceleration vectors for Target
Satellite trajectory in E-C-I frame. 
%}

x = state(1);
y = state(2);
z = state(3);

%%% Intertia parameters
m = 2.6;    % in kgs

%%% Kinematics
vel = state(4:6);

%%%Gravity Model
Reference_planet

r = state(1:3);
rnorm = norm(r);
rhat = r/rnorm;
Fgrav = -(G*m*M/rnorm^2)*rhat;
%%% Dynamics 
F = Fgrav;
accel = F/m;

%%% Return derivatives vector
dstatedt = [vel; accel];
