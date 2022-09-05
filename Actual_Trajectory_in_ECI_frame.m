%%% Simulation of Target and Chaser satellite trajectory in ECI frame. 
disp('Simulation started')

%%% Get planet parameters
Reference_planet

%%% Initial conditions for Target Satellite

%For now we're assuming that the chaser satellite has the information about
%the trajectory of target satellite. The target satellite is moving in an
%elliptical orbit.
 
alt_c = 10*254*1.6*1000; % in meters
R0 = Re+alt_c;           % in meters

x0_tar = R0; % in meters
y0_tar = 0;  % in meters
z0_tar = 0;  % in meters

i_tar = 0*pi/180;                                       % Inclination of target satellite orbit

r_tar = norm([x0_tar;y0_tar;z0_tar]);                   % Semi-major axis of target satellite
semi_major_tar = R0 + 9000*1000;
vcirc_tar = sqrt(2*myu/r_tar - myu/semi_major_tar);     % orbital speed of target satellite
%vcirc_tar = sqrt(myu/r_tar);
theta_tar = atan2(y0_tar,x0_tar);              

xdot0_tar = -vcirc_tar*sin(theta_tar);            % in m/s
ydot0_tar = vcirc_tar*cos(i_tar)*cos(theta_tar);  % in m/s
zdot0_tar = vcirc_tar*sin(i_tar)*cos(theta_tar);  % in m/s

stateinitial_tar = [x0_tar; y0_tar; z0_tar; xdot0_tar; ydot0_tar; zdot0_tar];  % initial state vector of target satellite
[a,ecc,incl,RAAN,argp,nu,truelon,arglat,lonper] = ijk2keplerian(stateinitial_tar(1:3),stateinitial_tar(4:6)); % Keplerian elements for target satellite 

%%% Initial conditions for Chaser Satellite 
alt_c = 10*254*1.6*1000; % in meters
R0 = Re + alt_c;         % in meters

x0_c = R0+500*1000;       % in meters
y0_c = 50*1000;           % in meters
z0_c = 20*1000;           % in meters

r_cha = norm([x0_c;y0_c;z0_c]);  % semi-major axis of chaser satellite
semi_major_c = R0+9000*1000;
vcirc_c = sqrt(2*myu/r_cha - myu/semi_major_c);       % orbital speed of target satellite
%vcirc_c = sqrt(myu/r_cha);
i_c = 0*pi/180;                         % inclination of chaser satellite orbit
theta_c = atan2(y0_c,x0_c);

xdot0_c = -vcirc_c*sin(theta_c);          % in m/s
ydot0_c = vcirc_c*cos(i_c)*cos(theta_c);  % in m/s
zdot0_c = vcirc_c*sin(i_c)*cos(theta_c);  % in m/s

stateinitial_c = [x0_c; y0_c; z0_c; xdot0_c; ydot0_c; zdot0_c];

%%% Time Window of target satellite
period = 2*pi/sqrt(myu)*semi_major_tar^(3/2);
%period = 2*pi/sqrt(myu)*r_tar^(3/2);
number_of_orbits = 1/4;
tspan = linspace(0,period*number_of_orbits,600);
%tspan = [0 period*number_of_orbits];

%{
%%% Integrating equations of motion
options = odeset('RelTol',1e-6);
%[tout_tar,stateout_tar] = ode45(@Target_Satellite,tspan,stateinitial_tar,options);  % target satellite equations 
[tout_c,stateout_c] = ode45(@Target_Satellite,tspan,stateinitial_c,options);
%}

%%% Conversion from ECI to LVLH frame
[C,Cdot] = ECI2LVLH(stateinitial_tar);

%%% Finding Initial conditions in LVLH frame
r_c_0 = stateinitial_c(1:3);
r_t_0 = stateinitial_tar(1:3);
r_ct_0_I = r_c_0 - r_t_0;             % position vector in ECI frame
r_ct_0_R = C*r_ct_0_I;              % position vector in rotating frame

%w = [0;0;vcirc_tar/semi_major_tar]; %orbital speed of target satellite
v_c_0 = stateinitial_c(4:6);
v_t_0 = stateinitial_tar(4:6);
v_ct_0_I = v_c_0 - v_t_0;
%v_ct_R = v_ct_I - cross(w,r_ct_R);
v_ct_0_R = Cdot*r_ct_0_I + C*v_ct_0_I;

%%% Finding Relative motion equations (CW_equations) 
Relative_Motion

%%Convert meters to Kilometers
stateout_tar = stateout_tar/1000;
%stateout_c = stateout_c/1000;

%%% Extract state vector of target satellite 
xout_tar = stateout_tar(:,1); % in kms
yout_tar = stateout_tar(:,2); % in kms
zout_tar = stateout_tar(:,3); % in kms

xdot_out_tar = stateout_tar(:,4)*1000; % in m/s
ydot_out_tar = stateout_tar(:,5)*1000; % in m/s
zdot_out_tar = stateout_tar(:,6)*1000; % in m/s

%{
%%% Extract state vector of chaser satellite 
xout_c = stateout_c(:,1);
yout_c = stateout_c(:,2);
zout_c = stateout_c(:,3);

xdot_out_c = stateout_c(:,4);
ydot_out_c = stateout_c(:,5);
zdot_out_c = stateout_c(:,6);

r_out_c = [xout_c,yout_c,zout_c];
v_out_c = [xdot_out_c,ydot_out_c,zdot_out_c];
%}

%%% Make an Earth
[X,Y,Z] = sphere();
X = X*Re/1000; %in Kms
Y = Y*Re/1000; % in Kms
Z = Z*Re/1000; % in Kms

%%% Plot 2D & 3D orbits
fig = figure();
set(fig,'color','white')  
subplot(1,2,1)
plot3(xout_tar, yout_tar, zout_tar,'b-')
xlabel('X (in kms)')
ylabel('Y (in kms)')
zlabel('Z (in kms)')
title('Absolute motions of chaser and target satellites in ECI frame')
grid on
hold on
plot3(xout_ECI,yout_ECI,zout_ECI,'r-')
%plot3(xout_c,yout_c,zout_c,'r-')
%plot(xout_c, yout_c)
t = plot3(xout_tar(1),yout_tar(1),zout_tar(1),'o','MarkerFaceColor','red'); %target satellite marker
c = plot3(xout_ECI(1),yout_ECI(1),zout_ECI(1),'o','MarkerFaceColor','green'); %chaser satellite marker
%c = plot3(xout_c(1),yout_c(1),zout_c(1),'o','MarkerFaceColor','green'); %chaser satellite marker

surf(X,Y,Z,'EdgeColor','none')
legend('Target Sat Trajectory','Chaser Sat Trajectory','Target Sat position','Chaser Sat position','Earth')
axis equal

subplot(1,2,2)
plot(xout_ECI/1000,yout_ECI/1000,'b-')
xlabel('X ( x 10^6 meters)')
ylabel('Y (x 10^6 meters)')
title('Absolute motion of chaser and target satellites in ECI frame (X-Y)')
grid on
hold on
%plot(xout_c/1000,yout_c/1000,'r-')
plot(xout_tar/1000,yout_tar/1000,'r-')
r = plot(xout_tar(1)/1000,yout_tar(1)/1000,'o','MarkerFaceColor','g');
r_R = plot(xout_ECI(1)/1000,yout_ECI(1)/1000,'o','MarkerFaceColor','y');
legend('Chaser Sat Trajectory', 'Target Sat Trajectory','Target Sat position','Chaser Sat position')
%axis equal

for k = 2:length(xout_ECI)
    t.XData = xout_tar(k);
    t.YData = yout_tar(k);
    t.ZData = zout_tar(k);
    c.XData = xout_ECI(k);
    c.YData = yout_ECI(k);
    c.ZData = zout_ECI(k);
    r.XData = xout_tar(k)/1000;
    r.YData = yout_tar(k)/1000;
    r_R.XData = xout_ECI(k)/1000;
    r_R.YData = yout_ECI(k)/1000;
    drawnow 
end

fig2 = figure();
set(fig,'color','white')
subplot(3,2,1)
plot(tout_ren,xout_ECI-xout_tar,'b-')
xlabel('Time (in seconds)')
ylabel('Error in X axis (in Kms)')
grid on

subplot(3,2,3) 
plot(tout_ren,yout_ECI-yout_tar,'b-')
xlabel('Time (in seconds)')
ylabel('Error in Y axis (in Kms)')
grid on

subplot(3,2,5)
plot(tout_ren,zout_ECI - zout_tar,'b-');
xlabel('Time (in seconds)')
ylabel('Error in Z axis (in Kms)')
grid on

subplot(3,2,2)
plot(tout_ren,xdot_out_ECI - xdot_out_tar,'b-');
xlabel('Time (in seconds)')
ylabel('Error in X axis (in m/s)')
grid on

subplot(3,2,4)
plot(tout_ren,ydot_out_ECI - ydot_out_tar,'b-');
xlabel('Time (in seconds)')
ylabel('Error in Y axis (in m/s)')
grid on

subplot(3,2,6)
plot(tout_ren,zdot_out_ECI - zdot_out_tar,'b-');
xlabel('Time (in seconds)')
ylabel('Error in Z axis (in m/s)')
grid on