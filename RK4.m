function [stateout,time] = RK4(timestep,tmax,stateinitial_nl,u,R0,w)
time = 0:timestep:tmax;
n = length(stateinitial_nl);
stateout = zeros(n,length(time)-1);
stateout(:,1) = stateinitial_nl;

%B = [0 0 0; 0 0 0; 0 0 0; 1 0 0; 0 1 0; 0 0 1];
%K = [1 0 0 0 0 0; 0 0.1 0 0 0 0; 0 0 0 0 0 0];

for i = 1:length(time)-1
    k1 = Derivatives(stateout(:,i),u(:,i),R0,w);
    k2 = Derivatives(stateout(:,i)+k1*timestep/2, u(:,i),R0,w);
    k3 = Derivatives(stateout(:,i)+k2*timestep/2,u(:,i),R0,w);
    k4 = Derivatives(stateout(:,i)+k3*timestep,u(:,i),R0,w);
    kRk4 = (1/6)*(k1+2*k2+2*k3+k4);

    %U(:,i) = -K*stateout(:,i);
    stateout(:,i+1) = stateout(:,i)+kRk4*timestep;
    
end