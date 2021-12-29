function out = multi_cycle_PID_test(PAR,opt)

ncycle = numel(opt.solution.phase);
const = PAR.const;

for q=1:ncycle
%% Propulsion phase
if rem(q,2)
	Vtau = [opt.solution.phase(q).control(:,1),opt.solution.phase(q).control(:,2),opt.solution.phase(q).control(:,5),opt.solution.phase(q).control(:,6)];
    q_ini = [opt.solution.phase(q).position(1,:)';opt.solution.phase(q).velocity(1,:)'];
    q_ini = [q_ini;opt.solution.phase(q).state(1,1);opt.solution.phase(q).state(1,2);opt.solution.phase(q).state(1,3);opt.solution.phase(q).state(1,4)];
	if q==1
		time = opt.solution.phase(q).time;
		[T1,Y1] = ode113(@(tv,vt) dynamics_int_erri(vt,interp1(time,Vtau,tv),PAR,const,1), time, q_ini, odeset('Events',@(tv,vt) EventsFcn(tv,vt,PAR,opt.solution.phase(q).position(end))));
	else
		time = opt.solution.phase(q).time-opt.solution.phase(q).time(1)+out.solution.phase(q-1).time(end);
		[T1,Y1] = ode113(@(tv,vt) dynamics_int_erri(vt,interp1(time,Vtau,tv),PAR,const,1), time, q_ini, odeset('Events',@(tv,vt) EventsFcn(tv,vt,PAR,opt.solution.phase(q).position(end))));
	end
	
	out.solution.phase(q).time = T1;
	alpha = [];
	beta = [];
	tp = [];
	tau = [];
	for i=1:size(Y1,1)
		theta = Y1(i,1);
		k1 = double(-2*PAR.B*(PAR.Y + PAR.R*sin(theta)));
		k2 = double(2*PAR.B*(-PAR.h - PAR.R*cos(theta)));
		k3 = double(- PAR.A^2 + PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2);
		beta = [beta;2*atan2(-k1-sqrt(k1^2+k2^2-k3^2),k3-k2)];
	
		k1a = -2*PAR.A*(PAR.Y + PAR.R*sin(theta));
		k2a = 2*PAR.A*(-PAR.h - PAR.R*cos(theta));
		k3a = PAR.A^2 - PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2;
		alpha = [alpha;2*atan2(-k1a+sqrt(k1a^2+k2a^2-k3a^2),k3a-k2a)];
		tp = [tp;dynamics_int_erri(Y1(i,:),interp1(time,Vtau,T1(i)),PAR,2e2,0)];
		tau = [tau;interp1(time,Vtau,T1(i))];
	end
	out.solution.phase(q).control = [tau(:,1:2),alpha,beta,tau(:,3:4),tp];
	out.solution.phase(q).position = Y1(:,1);
	out.solution.phase(q).velocity = Y1(:,2);
	out.solution.phase(q).state = Y1(:,3:6);
else
%% Recovery phase
    Vtau = [opt.solution.phase(q).control(:,1),opt.solution.phase(q).control(:,2),opt.solution.phase(q).control(:,3),opt.solution.phase(q).control(:,4)];
    time = opt.solution.phase(q).time-opt.solution.phase(q).time(1)+out.solution.phase(q-1).time(end);
    q_ini = [opt.solution.phase(q).position(1,:)';opt.solution.phase(q).velocity(1,:)';opt.solution.phase(q).state(1,:)'];
	[T2,Y2] = ode113(@(tv,vt) dynamics_int_errii(vt,interp1(time,Vtau,tv),PAR,const,1), time, q_ini);
	
	out.solution.phase(q).time = T2;
	out.solution.phase(q).position = Y2(:,1:3);
	out.solution.phase(q).velocity = Y2(:,4:6);
	out.solution.phase(q).state = Y2(:,7:10);
	out.solution.phase(q).control = Vtau;
end

% %% Final phase
% Vtau = [opt.solution.phase(2*q+1).control(:,1),opt.solution.phase(2*q+1).control(:,2)];
% time = opt.solution.phase(2*q+1).time-opt.solution.phase(2*q+1).time(1)+out.solution.phase(2*q).time(end);
% [T1,Y1] = ode113(@(tv,vt) dynamics_int_erri(vt,interp1(time,Vtau,tv),PAR,2e2,1),[time(1),time(end)],[opt.solution.phase(2*q+1).position(1);out.solution.phase(2*q).velocity(end);out.solution.phase(2*q).state(end,:)'],odeset('Events',@(tv,vt) EventsFcn(tv,vt,PAR,opt.solution.phase(2*q+1).position(end))));
% 
% out.solution.phase(2*q+1).time = T1;
% alpha = [];
% beta = [];
% tp = [];
% tau = [];
% for i=1:size(Y1,1)
%     theta = Y1(i,1);
%     k1 = double(-2*PAR.B*(PAR.Y + PAR.R*sin(theta)));
%     k2 = double(2*PAR.B*(-PAR.h - PAR.R*cos(theta)));
%     k3 = double(- PAR.A^2 + PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2);
%     beta = [beta;2*atan2(-k1-sqrt(k1^2+k2^2-k3^2),k3-k2)];
% 
%     k1a = -2*PAR.A*(PAR.Y + PAR.R*sin(theta));
%     k2a = 2*PAR.A*(-PAR.h - PAR.R*cos(theta));
%     k3a = PAR.A^2 - PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2;
%     alpha = [alpha;2*atan2(-k1a+sqrt(k1a^2+k2a^2-k3a^2),k3a-k2a)];
%     tp = [tp;dynamics_int_erri(Y1(i,:),interp1(time,Vtau,T1(i)),PAR,2e2,0)];
%     tau = [tau;interp1(time,Vtau,T1(i))];
% end
% out.solution.phase(2*q+1).control = [tau,alpha,beta,tp];
% out.solution.phase(2*q+1).position = Y1(:,1);
% out.solution.phase(2*q+1).velocity = Y1(:,2);
%  out.solution.phase(2*q+1).state = Y1(:,3:6);
end


out.Options = opt.Options;
out.solution.objective = opt.solution.objective;

%% Propulsion phase functions
function dy = dynamics_int_erri(y,u,PAR,const,flag)

Kte = const;
Jtotal = PAR.Jmt;
Btotal = PAR.Bt;
% delay = input.auxdata.PAR.delay;
Kmotor = PAR.rt*PAR.Kt;
Ke = PAR.rt*PAR.Ke;
Ra = PAR.Ra;
La = PAR.La;

theta = y(1);
    if theta<PAR.thetac1
        theta = PAR.thetac1;
    elseif theta>PAR.thetac2
        theta = PAR.thetac2;
    end
%     try
k1 = double(-2*PAR.B*(PAR.Y + PAR.R*sin(theta)));
k2 = double(2*PAR.B*(-PAR.h - PAR.R*cos(theta)));
k3 = double(- PAR.A^2 + PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2);
beta = 2*atan2(-k1-sqrt(k1^2+k2^2-k3^2),k3-k2);

k1a = -2*PAR.A*(PAR.Y + PAR.R*sin(theta));
k2a = 2*PAR.A*(-PAR.h - PAR.R*cos(theta));
k3a = PAR.A^2 - PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2;
alpha = 2*atan2(-k1a+sqrt(k1a^2+k2a^2-k3a^2),k3a-k2a);
%     catch
%         keyboard
%     end

dtheta = y(2);

V = y(3);
i_err = y(4);
i_motor = y(5);
d_err = y(6);
[dalpha,dbeta,PHI,dPHI] = idof_transform(alpha,beta,theta,dtheta,PAR);
q = [alpha,beta,theta];
qd = [dalpha,dbeta,dtheta];
err = V/PAR.Rr-dtheta;
[M,ke,~,H,k] = four_bar_system(q, qd,PAR);
MM = PHI'*M*PHI;
K = PHI'*M*dPHI*dtheta;
l_c = Kte*((Jtotal*Ra+La*Btotal)*err+(Btotal*Ra+Ke)*i_err+La*Jtotal*(1/PAR.Td)*(err-d_err))/Kmotor;
KE = PHI'*(H*[u(1)-u(3);u(2)-u(4)]+ke-k)+Kmotor*i_motor;
ddtheta = MM\(KE-K);
qdd = dPHI*dtheta+PHI*ddtheta;
tp = M(3,:)*qdd+k(3)-ke(3)-Kmotor*i_motor;
if flag ==1
	dy(3,1) = 1/PAR.Mi*(tp/PAR.R-PAR.Ci*V);
	% derr = dy(3,1)/PAR.Rr-ddtheta;
	dy(4,1) = err;
	dy(2,1) = MM\(KE-K);
	dy(1,1) = dtheta;
	dy(5,1) = (1/La)*(l_c-Ra*i_motor-Ke*dtheta);
	dy(6,1) = (1/PAR.Td)*(err-d_err);
else
    dy = tp;
end

function [position,isterminal,direction] = EventsFcn(t,y,PARAM,thetaf)

position = (y(1)-PARAM.thetac1)*(y(1)-thetaf); % The value that we want to be zero
isterminal = 1;  % Halt integration 
direction = 0;   % The zero can be approached from either direction

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Recovery phase functions
function dy = dynamics_int_errii(y,u,PAR,const,flag)

Kte = const;
Jtotal = PAR.Jmt;
Btotal = PAR.Bt;
% delay = input.auxdata.PAR.delay;
Kmotor = PAR.rt*PAR.Kt;
Ke = PAR.rt*PAR.Ke;
Ra = PAR.Ra;
La = PAR.La;

alpha = y(1);
beta  = y(2);
theta = y(3);

dalpha = y(4);
dbeta  = y(5);
dtheta = y(6);

V = y(7);
i_err = y(8);
i_motor = y(9);
d_err = y(10);

q = [alpha,beta,theta];
qd = [dalpha,dbeta,dtheta];
err = V/PAR.Rr-dtheta;
[M,ke,~,H,k] = four_bar_system(q, qd,PAR);
% MM = PHI'*M*PHI;
% K = PHI'*M*dPHI*dtheta;
l_c = Kte*((Jtotal*Ra+La*Btotal)*err+(Btotal*Ra+Ke)*i_err+La*Jtotal*(1/PAR.Td)*(err-d_err))/Kmotor;
KE = H*[u(1)-u(3);u(2)-u(4)]+ke-k+[0;0;Kmotor*i_motor];
qdd = M\KE;

dy(1,1) = dalpha;
dy(2,1) = dbeta;
dy(3,1) = dtheta;

dy(4,1) = qdd(1);
dy(5,1) = qdd(2);
dy(6,1) = qdd(3);

dy(7,1) = 1/PAR.Mi*(-PAR.Ci*V);
dy(8,1) = err;
dy(9,1) = (1/La)*(l_c-Ra*i_motor-Ke*dtheta);
dy(10,1) = (1/PAR.Td)*(err-d_err);
% else
%     dy = tp;
% end