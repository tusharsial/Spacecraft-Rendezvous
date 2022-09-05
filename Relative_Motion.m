%%% Simulation of Relative motion (Satellite Rendezvous)
disp('Relative Motion Simulation started')

%%% Initial conditions
%w = vcirc_tar/semi_major_tar;  % orbital velocity of target satellite
w = sqrt(myu/semi_major_tar^3); %orbital rate of target satellite
%w = sqrt(myu/r_tar^3);

%%% Extracting initial states of chaser satellite in rotating coordinate frame (LVLH) 
x0 = r_ct_0_R(1); % in m
y0 = r_ct_0_R(2); % in m
z0 = r_ct_0_R(3); % in m 

xdot0 = v_ct_0_R(1); % in m/s
ydot0 = v_ct_0_R(2); % in m/s
zdot0 = v_ct_0_R(3); % in m/s

stateinitial = [x0; y0; z0; xdot0; ydot0; zdot0]; % Initial state vector of chaser satellite in LVLH frame

%%% Integrating equations of motion
options = odeset('RelTol',1e-6);
%[tout_ren,stateout_ren] = ode45(@Rendezvous,tspan,stateinitial,options);  %Relative motion equation
[tout_tar,stateout_tar] = ode45(@Target_Satellite,tspan,stateinitial_tar,options);
[tout_ren,stateout_ren,U] = ode45(@Rendezvous_with_Perturbation,tspan,stateinitial,options,w,R0);

%%% Time Window
N = 500;
m = 3;
tmax = 2*period*number_of_orbits;
timestep = tmax/N;
time = 0:timestep:tmax;

%%%Converting Continuous to discrete using Runga Kutte Method 
stateinitial_test = [1000;-1000;1000;3;3;-3];
N_test = 500;
tmax_test = 5000;
timestep_test = tmax_test/N;
time_test = 0:timestep_test:tmax_test;

for i = 1:length(time_test)-1
    utest(1,i) = unifrnd(-1,1);
    utest(2,i) = unifrnd(-1,1);
    utest(3,i) = unifrnd(-1,1);
end

[stateout_ren_dis_test,time_test] = RK4(timestep_test,tmax_test,stateinitial_test,utest,R0,w);

%[stateout_ren_dis,time] = RK4(timestep,tmax,stateinitial,R0,w);

%%% Koopman Transformation 
stateout_koop_test = Koopman(stateout_ren_dis_test);
%stateout_koop = Koopman(stateout_ren_dis);

%%% Naive DMD
[G, Akoop, Bkoop] = DMD(stateout_ren_dis_test,utest);
%Z = DMD(stateout_koop,time); %output state vector in LVLH frame after DMD

%%% Calculating Terminal state vector in Lifted Dimension
[r,c] = size(Bkoop);

for i = 1:N
    CN_koop(:,c*i-2:c*i) = Akoop^(N-i)*Bkoop;
end

beta_koop = (Akoop^N)*stateout_koop_test(:,1);

%%% IRLS Algorithm for finding optimal control inputs
%ukoop = IRLS(N,m,CN_koop,beta_koop);

%%% Constructing Final output state vector
%time_test = 0:timestep:tmax_test-1;

%Q = 100*[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;0 0 0 1000 0 0; 0 0 0 0 1000 0; 0 0 0 0 0 1000;];
%R  = 10^7*eye(3);

Q = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;0 0 0 100 0 0; 0 0 0 0 100 0; 0 0 0 0 0 100;];
R  = 5*10^8*eye(3);

K = dlqr(Akoop,Bkoop,Q,R);

Z = zeros(length(stateout_ren_dis_test(:,1)),length(time));
Z(:,1) = stateinitial;
for i = 1:length(time)-1
    utest(:,i) = -K*Z(:,i);
    Z(:,i+1) = Akoop*Z(:,i) + Bkoop*utest(:,i);
end

plot(time,Z(1,:));
%plot(time_test(1:end-1),utest(3,:));


%%% Extracting output state vector in rotating frame (LVLH)

xout = stateout_ren(:,1);
yout = stateout_ren(:,2);
zout = stateout_ren(:,3);

xdot_out = stateout_ren(:,4);
ydot_out = stateout_ren(:,5);
zdot_out = stateout_ren(:,6);

%{
xout = stateout_ren_dis(1,:);
yout = stateout_ren_dis(2,:);
zout = stateout_ren_dis(3,:);

xdot_out = stateout_ren_dis(4,:);
ydot_out = stateout_ren_dis(5,:);
zdot_out = stateout_ren_dis(6,:);
%}

xout_tar = stateout_tar(:,1);
yout_tar = stateout_tar(:,2);
zout_tar = stateout_tar(:,3);

xdot_out_tar = stateout_tar(:,4);
ydot_out_tar = stateout_tar(:,5);
zdot_out_tar = stateout_tar(:,6);


%%% Converting from LVLH frame to ECI frame
r_out_LVLH = [xout,yout,zout];
%r_out_LVLH = [xout;yout;zout];
r_out_tar = [xout_tar,yout_tar,zout_tar];

v_out_LVLH = [xdot_out,ydot_out,zdot_out];
v_out_tar = [xdot_out_tar,ydot_out_tar,zdot_out_tar];

[r_out_ECI,v_out_ECI] = LVLH2ECI(C,Cdot,r_out_LVLH,r_out_tar,v_out_LVLH,v_out_tar);

%%% Extracting output position vector of chaser satellite in ECI frame
xout_ECI = r_out_ECI(:,1)/1000; %in Kms
yout_ECI = r_out_ECI(:,2)/1000; %in Kms
zout_ECI = r_out_ECI(:,3)/1000; %in Kms

xdot_out_ECI = v_out_ECI(:,1); %in m/s
ydot_out_ECI = v_out_ECI(:,2); %in m/s
zdot_out_ECI = v_out_ECI(:,3); %in m/s

%%% Practice plots for testing 
fig3 = figure();
set(fig3,'color','white');  

subplot(3,2,1)
plot(time,Z(1,:)/1000,'b-')
xlabel('Time (in seconds)')
ylabel('Error in X axis (in Kms)')
grid on

subplot(3,2,3) 
plot(time,Z(2,:)/1000,'b-')
xlabel('Time (in seconds)')
ylabel('Error in Y axis (in Kms)')
grid on

subplot(3,2,5)
plot(time,Z(3,:)/1000,'b-');
xlabel('Time (in seconds)')
ylabel('Error in Z axis (in Kms)')
grid on

subplot(3,2,2)
plot(time,Z(4,:),'b-');
xlabel('Time (in seconds)')
ylabel('Error in X axis (in m/s)')
grid on

subplot(3,2,4)
plot(time,Z(5,:),'b-');
xlabel('Time (in seconds)')
ylabel('Error in Y axis (in m/s)')
grid on

subplot(3,2,6)
plot(time,Z(6,:),'b-');
xlabel('Time (in seconds)')
ylabel('Error in Z axis (in m/s)')
grid on

%ylabel('X in LVLH frame (in km)')
%plot(time,stateout_ren_dis(1,:)- Z(1,:))     % to plot error between actual signal and DMD signal

%plot(tout_ren,zout,'b-');
%hold on 
%plot(time,stateout_ren_dis(3,:),'r-')


