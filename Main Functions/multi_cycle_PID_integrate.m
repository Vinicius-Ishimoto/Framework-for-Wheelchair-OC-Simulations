function out = multi_cycle_PID_integrate(PAR,opt)

ncycle = numel(opt.solution.phase);
const = PAR.const;

for q=1:ncycle
%% Propulsion phase
if rem(q,2)
	Vtau = [opt.solution.phase(q).control(:,1),opt.solution.phase(q).control(:,2),opt.solution.phase(q).control(:,5),opt.solution.phase(q).control(:,6)];
	if q==1
		time = opt.solution.phase(q).time;
        rtheta = opt.solution.phase(q).position(:,1);
		[T1,Y1] = ode113(@(tv,vt) dynamics_int_erri(vt,interp1(rtheta,Vtau,vt(1),'spline','extrap'),PAR,const,1),[time(1),time(end)],[opt.solution.phase(q).position(1);opt.solution.phase(q).velocity(1);opt.solution.phase(q).velocity(1)*PAR.Rr;0;0;0],odeset('Events',@(tv,vt) EventsFcn(tv,vt,PAR,opt.solution.phase(q).position(end))));
	else
		time = opt.solution.phase(q).time-opt.solution.phase(q).time(1)+out.solution.phase(q-1).time(end);
        rtheta = opt.solution.phase(q).position(:,1);
		[T1,Y1] = ode113(@(tv,vt) dynamics_int_erri(vt,interp1(rtheta,Vtau,vt(1),'spline','extrap'),PAR,const,1),[time(1),time(end)],[opt.solution.phase(q).position(1);out.solution.phase(q-1).velocity(end);	out.solution.phase(q-1).state(end,:)'],odeset('Events',@(tv,vt) EventsFcn(tv,vt,PAR,opt.solution.phase(q).position(end))));
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
		tp = [tp;dynamics_int_erri(Y1(i,:),interp1(rtheta,Vtau,Y1(i,1),'spline','extrap'),PAR,2e2,0)];
		tau = [tau;interp1(rtheta,Vtau,Y1(i,1),'spline','extrap')];
	end
	out.solution.phase(q).control = [tau(:,1:2),alpha,beta,tau(:,3:4),tp];
	out.solution.phase(q).position = Y1(:,1);
	out.solution.phase(q).velocity = Y1(:,2);
	out.solution.phase(q).state = Y1(:,3:6);
    out.solution.phase(q).taup = tp;
else
%% Recovery phase
	out.solution.phase(q).position = opt.solution.phase(q).position;
	Y2 = dynamics_int_errii(Y1,T1,PAR,const,out,q,opt);
	
	out.solution.phase(q).time = Y2.solution.phase.time;
	out.solution.phase(q).position = Y2.solution.phase.position;
	out.solution.phase(q).velocity = Y2.solution.phase.velocity;
	out.solution.phase(q).state = Y2.solution.phase.state;
	out.solution.phase(q).control = Y2.solution.phase.control;
    out.solution.phase(q).taup = zeros(numel(out.solution.phase(q).control(:,1)),1);
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
out.Options = opt.Options;
out.solution.objective = inf;
out.Options.ImpedanceMass = PAR.Mi;
out.Options.ImpedanceFriction = PAR.Ci;
end


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
	dy(3,1) = 1/PAR.Mi*(tp/PAR.R-PAR.Ci*V-PAR.Fri);
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
function dy = dynamics_int_errii(y,u,PAR,const,opt,q,rg)

PAR.thetai = opt.solution.phase(q).position(end,3);
setup.auxdata.PAR = PAR;
setup.mesh_points = 10;
setup.mesh_number = 8;
setup.auxdata.const = const;
setup.function = @Dynamics_ii;
dT = rg.solution.phase(q).time(end)-rg.solution.phase(q).time(1);

setup.bound.lower.phase.initial.position = [opt.solution.phase(q-1).control(end,3:4),opt.solution.phase(q-1).position(end)];
setup.bound.lower.phase.initial.velocity = [-10*pi,-10*pi,opt.solution.phase(q-1).velocity(end)];
setup.bound.lower.phase.initial.state = opt.solution.phase(q-1).state(end,:);
setup.bound.lower.phase.initial.time = opt.solution.phase(q-1).time(end);
setup.bound.lower.phase.final.position = [opt.solution.phase(q).position(end,1:2),PAR.thetac1];
setup.bound.lower.phase.final.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase.final.state = [0,-inf,-inf,-inf];
setup.bound.lower.phase.final.time = opt.solution.phase(q-1).time(end)+dT-0.01;
setup.bound.lower.phase.position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase.state = [0,-inf,-inf,-inf];
setup.bound.lower.phase.control = [0,0,0,0];
setup.bound.lower.phase.path = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.upper.phase.initial.position = [opt.solution.phase(q-1).control(end,3:4),opt.solution.phase(q-1).position(end)];
setup.bound.upper.phase.initial.velocity = [10*pi,10*pi,opt.solution.phase(q-1).velocity(end)];
setup.bound.upper.phase.initial.state = opt.solution.phase(q-1).state(end,:);
setup.bound.upper.phase.initial.time = opt.solution.phase(q-1).time(end);
setup.bound.upper.phase.final.position = [opt.solution.phase(q).position(end,1:2),inf];
% setup.bound.upper.phase.final.position = [opt.solution.phase(q).position(end,1:2),inf];
setup.bound.upper.phase.final.velocity = [10*pi,10*pi,50];
setup.bound.upper.phase.final.state = [inf,inf,inf,inf];
setup.bound.upper.phase.final.time = dT+opt.solution.phase(q-1).time(end);
setup.bound.upper.phase.position = [pi,pi,inf];
setup.bound.upper.phase.velocity = [10*pi,10*pi,50];
setup.bound.upper.phase.state = [inf,inf,inf,inf];
setup.bound.upper.phase.control = inf(1,4);
setup.bound.upper.phase.path = pi;
setup.bound.lower.pconstraints = [0,0,0,0];
setup.bound.upper.pconstraints = [0,0,0,0];

%% Guess

setup.initial_guess.phase.time = rg.solution.phase(q).time;
setup.initial_guess.phase.position = rg.solution.phase(q).position;
setup.initial_guess.phase.velocity = rg.solution.phase(q).velocity;
setup.initial_guess.phase.state = [rg.solution.phase(q).velocity*PAR.Rr,rg.solution.phase(q).control(:,1:2),rg.solution.phase(q).control(:,1:2),rg.solution.phase(q).velocity];
setup.initial_guess.phase.control = 0.1*rg.solution.phase(q).control;

dy = ddiopt_MB(setup);

function output = Dynamics_ii(input)

Kte = input.auxdata.const;
Jtotal = input.auxdata.PAR.Jmt;
Btotal = input.auxdata.PAR.Bt;
% delay = input.auxdata.PAR.delay;
Kmotor = input.auxdata.PAR.rt*input.auxdata.PAR.Kt;
Ke = input.auxdata.PAR.rt*input.auxdata.PAR.Ke;
Ra = input.auxdata.PAR.Ra;
La = input.auxdata.PAR.La;
Mi = input.auxdata.PAR.Mi;
Ci = input.auxdata.PAR.Ci;
Fi = input.auxdata.PAR.Fri;

q2 = input.phase.position;
qd2 = input.phase.velocity;
tau_s2 = input.phase.control(:,1)-input.phase.control(:,3);
tau_e2 = input.phase.control(:,2)-input.phase.control(:,4);
v_imp2 = input.phase.state(:,1);
i_err2 = input.phase.state(:,2);
d_err2 = input.phase.state(:,4);
i_motor2 = input.phase.state(:,3);
err2 = v_imp2/input.auxdata.PAR.Rr-qd2(3);

[M2,ke2,~,H2,k2] = four_bar_system(q2, qd2,input.auxdata.PAR);
l_c2 = Kte*((Jtotal*Ra+La*Btotal)*err2+(Btotal*Ra+Ke)*i_err2+La*Jtotal*(1/input.auxdata.PAR.Td)*(err2-d_err2))/Kmotor;
output.phase.RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2 + [0;0;Kmotor*i_motor2];
output.phase.MassMatrix = M2;
output.phase.path = q2(:,1)-q2(:,2);

ddx2 = (1/Mi)*(-Ci*v_imp2-Fi);
output.phase.derivatives = [ddx2,err2, ...
    (1/La)*(l_c2-Ke*qd2(3)-Ra*i_motor2), (1/input.auxdata.PAR.Td)*(err2-d_err2)];

[dalphai,dbetai] = idof_transform(input.phase.initial.position(1),input.phase.initial.position(2),input.phase.initial.position(3),input.phase.initial.velocity(3),input.auxdata.PAR);
[dalphaf,dbetaf] = idof_transform(input.phase.final.position(1),input.phase.final.position(2),input.auxdata.PAR.thetai,input.phase.final.velocity(3),input.auxdata.PAR);

output.constraints = [dalphai-input.phase.initial.velocity(1),dbetai-input.phase.initial.velocity(2), ...
    dalphaf-input.phase.final.velocity(1),dbetaf-input.phase.final.velocity(2)];

obj = integrate(input.phase.integrand,input.phase.control(:,1).^2+input.phase.control(:,2).^2);
obj = obj + integrate(input.phase.integrand,input.phase.control(:,3).^2+input.phase.control(:,4).^2);
output.objective = obj;