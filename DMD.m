function [G, A, B] = DMD(stateout_ren_dis_test,U,time)

%This function performs DMD on the discretized system.

X = stateout_ren_dis_test(:,1:end-1);
Y = stateout_ren_dis_test(:,2:end);

%A = Y*pinv(X);
%B = eye(6);

G = Y*pinv([X;U]);

A = G(:,1:6);
B = G(:,7:end);
%{
%Q = eye(6);
Q = 10^-3*[1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0;0 0 0 10000 0 0; 0 0 0 0 100 0; 0 0 0 0 0 100;];
R  = 10^8*eye(6);
K = dlqr(A,B,Q,R);


Z = zeros(length(stateout_ren_dis_test(:,1)),length(time));
Z(:,1) = stateout_ren_dis_test(:,1);
for i = 1:length(time)-1
    %U = -K*Z(:,i);
    Z(:,i+1) = A*Z(:,i) + B*U;
end

%}