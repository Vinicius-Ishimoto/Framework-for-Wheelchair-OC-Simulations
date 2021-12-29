function [dalpha,dbeta,PHI,dPHI] = idof_transform(alpha,beta,theta,dtheta,PAR)

A = PAR.A;
B = PAR.B;
R = PAR.R;

iJ =  [(R*sin(beta - theta))./(A*sin(alpha - beta))
        -(R*sin(alpha - theta))./(B*sin(alpha - beta))];
dq = -iJ*dtheta;
dalpha = dq(1);
dbeta = dq(2);

PHI = [-iJ;1];

dPHI = [ -(R*(dtheta*sin(alpha - 2*beta + theta) - dalpha*sin(alpha - 2*beta + theta) + dalpha*sin(alpha - theta) - 2*dbeta*sin(alpha - theta) + dtheta*sin(alpha - theta)))./(A*(cos(2*alpha - 2*beta) - 1)); ...
     -(R*(dbeta*sin(beta - theta) - 2*dalpha*sin(beta - theta) + dtheta*sin(beta - theta) + dbeta*sin(2*alpha - beta - theta) - dtheta*sin(2*alpha - beta - theta)))./(B*(cos(2*alpha - 2*beta) - 1)); ...
        0];
    
end