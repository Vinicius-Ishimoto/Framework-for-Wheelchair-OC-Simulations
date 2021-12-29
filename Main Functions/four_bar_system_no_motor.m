function [M1,ke1,G,H,k] = four_bar_system_no_motor(q,dq,PARAM)

% M1*qdd = ke1 + G*[Fx; Fy] + H*[tau_s; tau_e] - k
% Returns the matrices used to compute the accelerations of the four-bar
% system.
% q = [alpha,beta,theta];
% dq = [dalpha,dbeta,dtheta];
% A*cos(alpha)+B*cos(beta)-R*cos(theta) == h
% A*sin(alpha)+B*sin(beta)-R*sin(theta) == v
%% Return Information from PARAM
ma = PARAM.ma; % [kg] massa dos ante-braços + mãos
mb = PARAM.mb; % [kg] massa dos braços
mc = PARAM.mc; % [kg] massa do corpo (tronco + pernas + cabeça)
h = PARAM.h;
Y = PARAM.Y;
A = PARAM.A;
B = PARAM.B;
a = PARAM.a; % [m] distância do CG do antebraço + mãos # cotovelo
b = PARAM.b; % [m] distância do CG do braço # ombro
Ja = PARAM.Ja; % [kg*m^2] momento de inércia dos dois antebraço + mãos
Jb = PARAM.Jb; % [kg*m^2] momento de inércia dos dois braços
mr = PARAM.mr; % [kg] massa das duas rodas da cadeira
R = PARAM.R; % [m] raio do aro de propulsão (diâmetro de 22 polegadas = 0.5588 m)
Rr = PARAM.Rr; % [m] raio da roda da cadeira
Jr = PARAM.Jr; % [kg*m^2] momento de inércia das duas rodas
mw = PARAM.mw; % [kg] massa do corpo + cadeira
dFric = PARAM.dFric;
Frr = PARAM.Frr;
ni = PARAM.Ang;

%% Parâmetros do conjunto
alpha = q(1);
beta = q(2);
theta = q(3);
dalpha = dq(1);
dbeta = dq(2);
dtheta = dq(3);

%% Matrizes do sistema
M1 = [         mb*A^2 + ma*a^2 + Ja, A*b*mb*cos(alpha - beta),               -Rr*sin(alpha)*(A*mb + a*ma);
     A*b*mb*cos(alpha - beta),              mb*b^2 + Jb,                         -Rr*b*mb*sin(beta);
 -Rr*sin(alpha)*(A*mb + a*ma),       -Rr*b*mb*sin(beta), Jr + Rr^2*ma + Rr^2*mb + Rr^2*mr + Rr^2*mw];

k =  [                                                      A*b*dbeta.^2*mb*sin(alpha - beta);
                                                           -A*dalpha.^2*b*mb*sin(alpha - beta);
 - Rr*b*mb*cos(beta)*dbeta.^2 - dalpha*(A*Rr*dalpha*mb*cos(alpha) + Rr*a*dalpha*ma*cos(alpha))];
 
% ke1 = [   A*cos(alpha)*((981*mb)/100) + (981*a*ma*cos(alpha))/100;
%                           (981*b*mb*cos(beta))/100;
%                            tp - Frr*Rr - Rr*dFric*dtheta - Rr*PARAM.Mf*9.81*sin(PARAM.Ang)];
                       
% ke1 = [   (981*a*ma*cos(alpha - ni))/100 + (981*A*mb*cos(alpha - ni))/100;
%                            (981*b*mb*cos(beta - ni))/100;
%                             - Frr*Rr*tanh(1e4*dtheta) - Rr^2*dFric*dtheta - (981*Rr*(ma+mb+mr+mw)*sin(ni))/100];
ke1 = [   (981*a*ma*cos(alpha - ni))/100 + (981*A*mb*cos(alpha - ni))/100;
                           (981*b*mb*cos(beta - ni))/100;
                            - Frr*Rr - Rr^2*dFric*dtheta - (981*Rr*(ma+mb+mr+mw)*sin(ni))/100];
                       
H = [1 -1;0 1; 0 0];
G = [A*sin(alpha),-A*cos(alpha); B*sin(beta),- B*cos(beta);- R*sin(theta),R*cos(theta)];

end