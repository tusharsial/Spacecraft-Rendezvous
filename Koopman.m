function stateout_koop = Koopman(stateout_ren_dis)

g1 = stateout_ren_dis(1,:);
g2 = stateout_ren_dis(2,:);
g3 = stateout_ren_dis(3,:);
g4 = stateout_ren_dis(4,:);
g5 = stateout_ren_dis(5,:);
g6 = stateout_ren_dis(6,:);

%{
g7 = 1/(1 + (g1)^2 + (g2)^2 + (g3)^2)^3/2;
g8 = g1/(1 + (g1)^2 + (g2)^2 + (g3)^2)^3/2;
g9 = g2/(1 + (g1)^2 + (g2)^2 + (g3)^2)^3/2;
g10 = g3/(1 + (g1)^2 + (g2)^2 + (g3)^2)^3/2;
g11 = g4/(1 + (g1)^2 + (g2)^2 + (g3)^2)^3/2;
g12 = g5/(1 + (g1)^2 + (g2)^2 + (g3)^2)^3/2;
g13 = g6/(1 + (g1)^2 + (g3)^2 + (g3)^2)^3/2; 
%}

stateout_koop = [g1;g2;g3;g4;g5;g6];
%stateout_koop = [g1;g2;g3;g4;g5;g6;g7;g8;g9;g10;g11;g12;g13];