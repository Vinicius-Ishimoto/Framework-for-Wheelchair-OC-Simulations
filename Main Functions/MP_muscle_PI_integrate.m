function out = MP_muscle_PI_integrate(PAR,opt,OnlyPos)

ncycle = numel(opt.solution.phase);
const = PAR.const;
PAR.OnlyPos = OnlyPos;

for q=1:ncycle
%% Propulsion phase
if rem(q,2)
	% Muscle activation act[se,sf,ee,ef]
	rtheta = opt.solution.phase(q).position(:,1);
	Vtau = [opt.solution.phase(q).control(:,1),opt.solution.phase(q).control(:,5),opt.solution.phase(q).control(:,2),opt.solution.phase(q).control(:,6)];
	if q==1
		time = opt.solution.phase(q).time;
		[T1,Y1] = ode113(@(tv,vt) dynamics_int_erri(vt,interp1(rtheta,Vtau,vt(1),'spline','extrap'),PAR,const,1),[time(1),time(end)],[opt.solution.phase(q).position(1);opt.solution.phase(q).velocity(1);opt.solution.phase(q).velocity(1)*PAR.Rr;0],odeset('Events',@(tv,vt) EventsFcn(tv,vt,PAR,opt.solution.phase(q).position(end))));
	else
		time = opt.solution.phase(q).time-opt.solution.phase(q).time(1)+out.solution.phase(q-1).time(end);
		[T1,Y1] = ode113(@(tv,vt) dynamics_int_erri(vt,interp1(rtheta,Vtau,vt(1),'spline','extrap'),PAR,const,1),[time(1),time(end)],[opt.solution.phase(q).position(1);out.solution.phase(q-1).velocity(end);	out.solution.phase(q-1).state(end,:)'],odeset('Events',@(tv,vt) EventsFcn(tv,vt,PAR,opt.solution.phase(q).position(end))));
	end
	
	out.solution.phase(q).time = T1;
	alpha = [];
	beta = [];
	tp = [];
    	ts_e = [];
	ts_f = [];
	te_e = [];
	te_f = [];
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

        	tarray = dynamics_int_erri(Y1(i,:),interp1(rtheta,Vtau,Y1(i,1),'spline','extrap'),PAR,const,0);
		tp = [tp;tarray(1)];
        	ts_e = [ts_e;tarray(2)];
        	ts_f = [ts_f;tarray(3)];
        	te_e = [te_e;tarray(4)];
		te_f = [te_f;tarray(5)];
		tau = [tau;interp1(rtheta,Vtau,Y1(i,1),'spline','extrap')];
	end
	out.solution.phase(q).control = [tau(:,1),tau(:,3),alpha,beta,tau(:,2),tau(:,4),tp,ts_e,te_e,ts_f,te_f];
	out.solution.phase(q).position = Y1(:,1);
	out.solution.phase(q).velocity = Y1(:,2);
	out.solution.phase(q).state = Y1(:,3:4);
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
out.Options.Type = 'PI';
end


%% Propulsion phase functions
function dy = dynamics_int_erri(y,u,PAR,const,flag)

Kte = const;
Jtotal = PAR.Jmf; % Jmt - C/Motor; Jmf - S/Motor
Btotal = PAR.Bf+0.01; % Bt - C/Motor; Bf - S/Motor
% delay = input.auxdata.PAR.delay;
Kmotor = PAR.rt*PAR.Kt;
Ke = PAR.rt*PAR.Ke;
Ra = PAR.Ra;
La = PAR.La;

%% Angles
theta = y(1);
    if theta<PAR.thetac1
        theta = PAR.thetac1;
    elseif theta>PAR.thetac2
        theta = PAR.thetac2;
    end

k1 = double(-2*PAR.B*(PAR.Y + PAR.R*sin(theta)));
k2 = double(2*PAR.B*(-PAR.h - PAR.R*cos(theta)));
k3 = double(- PAR.A^2 + PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2);
beta = 2*atan2(-k1-sqrt(k1^2+k2^2-k3^2),k3-k2);

k1a = -2*PAR.A*(PAR.Y + PAR.R*sin(theta));
k2a = 2*PAR.A*(-PAR.h - PAR.R*cos(theta));
k3a = PAR.A^2 - PAR.B^2 + PAR.R^2 + 2*sin(theta)*PAR.R*PAR.Y + 2*cos(theta)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2;
alpha = 2*atan2(-k1a+sqrt(k1a^2+k2a^2-k3a^2),k3a-k2a);

%% Angular Velocity
dtheta = y(2);
[dalpha,dbeta,PHI,dPHI] = idof_transform(alpha,beta,theta,dtheta,PAR);

%% Activation to Torque
[ap1, bt1] = dof_conversion(alpha,beta,'Type', 'angle');
[dap1, dbt1] = dof_conversion(dalpha,dbeta,'Type', 'angvel');
T = muscle_torque_generators(u(2),u(1),u(4),u(3),ap1,bt1,dap1,dbt1);

V = y(3);
i_err = y(4);

q = [alpha,beta,theta];
qd = [dalpha,dbeta,dtheta];
err = V/PAR.Rr-dtheta;
[M,ke,~,H,k] = four_bar_system(q, qd,PAR,'Type','Reference');
MM = PHI'*M*PHI;
K = PHI'*M*dPHI*dtheta;
% l_c = Kte*((Jtotal*Ra+La*Btotal)*err+(Btotal*Ra+Ke)*i_err+La*Jtotal*(1/PAR.Td)*(err-d_err))/Kmotor;
l_c = Kte*(Jtotal*err+Btotal*i_err);
% if (PAR.OnlyPos)
%     U = max(0,l_c);
% else
    U = l_c;
% end
KE = PHI'*(H*[T(2)-T(1);T(4)-T(3)]+ke-k)+U;
ddtheta = MM\(KE-K);
qdd = dPHI*dtheta+PHI*ddtheta;
tp = M(3,:)*qdd+k(3)-ke(3)-U;
if flag ==1
	dy(3,1) = 1/PAR.Mi*(tp/PAR.R-PAR.Ci*V-PAR.Fri);
	% derr = dy(3,1)/PAR.Rr-ddtheta;
	dy(4,1) = err;
	dy(2,1) = MM\(KE-K);
	dy(1,1) = dtheta;
else
    dy(1,1) = tp;
    dy(2,1) = -T(2);
    dy(3,1) = T(1);
    dy(4,1) = -T(4);
    dy(5,1) = T(3);
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
acv_v = 10;

%[dap,dbt] = idof_transform(opt.solution.phase(q-1).control(end,3),opt.solution.phase(q-1).control(end,4),opt.solution.phase(q-1).position(end),opt.solution.phase(q-1).velocity(end),opt.auxdata.PAR);

setup.bound.lower.phase.initial.position = [opt.solution.phase(q-1).control(end,3:4),opt.solution.phase(q-1).position(end)];
setup.bound.upper.phase.initial.position = [opt.solution.phase(q-1).control(end,3:4),opt.solution.phase(q-1).position(end)];
setup.bound.lower.phase.initial.velocity = [-10*pi,-10*pi,opt.solution.phase(q-1).velocity(end)];
setup.bound.upper.phase.initial.velocity = [ 10*pi, 10*pi,opt.solution.phase(q-1).velocity(end)];
setup.bound.lower.phase.initial.state = opt.solution.phase(q-1).state(end,:);
setup.bound.upper.phase.initial.state = opt.solution.phase(q-1).state(end,:);
setup.bound.lower.phase.initial.time = opt.solution.phase(q-1).time(end);
setup.bound.upper.phase.initial.time = opt.solution.phase(q-1).time(end);

setup.bound.lower.phase.final.position = [opt.solution.phase(q).position(end,1:2),PAR.thetac1];
setup.bound.upper.phase.final.position = [opt.solution.phase(q).position(end,1:2),inf];
setup.bound.lower.phase.final.velocity = [-10*pi,-10*pi,  0];
setup.bound.upper.phase.final.velocity = [ 10*pi, 10*pi,inf];
setup.bound.lower.phase.final.state = [  0,-inf];
setup.bound.upper.phase.final.state = [inf, inf];
setup.bound.lower.phase.final.time = opt.solution.phase(q-1).time(end)+dT-0.01;
setup.bound.upper.phase.final.time = dT+opt.solution.phase(q-1).time(end);

setup.bound.lower.phase.position = [ 30*pi/180,-pi/2,PAR.thetac1];
setup.bound.upper.phase.position = [160*pi/180,   pi,inf];
setup.bound.lower.phase.velocity = [-10*pi,-10*pi,0];
setup.bound.upper.phase.velocity = [ 10*pi, 10*pi,50];
setup.bound.lower.phase.state = [0,-inf];
setup.bound.upper.phase.state = [inf,inf];
setup.bound.lower.phase.control = [0,0,0,0,  0,  0,  0,  0];
setup.bound.upper.phase.control = [1,1,1,1,inf,inf,inf,inf];
setup.bound.lower.phase.path = [         0,-acv_v,-acv_v,-acv_v,-acv_v];
setup.bound.upper.phase.path = [120*pi/180,+acv_v,+acv_v,+acv_v,+acv_v];

setup.bound.lower.pconstraints = [0,0,0,0];
setup.bound.upper.pconstraints = [0,0,0,0];

%% Guess

setup.initial_guess.phase.time = rg.solution.phase(q).time;
setup.initial_guess.phase.position = rg.solution.phase(q).position;
setup.initial_guess.phase.velocity = rg.solution.phase(q).velocity;
setup.initial_guess.phase.state = [rg.solution.phase(q).velocity(:,3)*PAR.Rr,rg.solution.phase(q).control(:,1)];
if size(rg.solution.phase(q).control,2) < 7
    setup.initial_guess.phase.control = [0.1*rg.solution.phase(q).control,0.5*rg.solution.phase(q).control(:,1:3)];
else
    setup.initial_guess.phase.control = rg.solution.phase(q).control;
end

dy = ddiopt_MB(setup);

function output = Dynamics_ii(input)

Kte = input.auxdata.const;
Jtotal = input.auxdata.PAR.Jmf; % Jmt - C/Motor; Jmf - S/Motor
Btotal = input.auxdata.PAR.Bf+0.01; % Bt - C/Motor; Bf - S/Motor
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

%% Activation
as_e  = input.phase.control(:,1);
as_f  = input.phase.control(:,3);
ae_e  = input.phase.control(:,2);
ae_f  = input.phase.control(:,4);
[ap1, bt1] = dof_conversion(q2(1),q2(2),'Type', 'angle');
[dap1, dbt1] = dof_conversion(qd2(1),qd2(2),'Type', 'angvel');
[~,~,TA,TP] = MTG_smooth(as_f,as_e,ae_f,ae_e,ap1,bt1,dap1,dbt1);

%% Torque
ts_e  = input.phase.control(:,5);
ts_f  = input.phase.control(:,7);
te_e  = input.phase.control(:,6);
te_f  = input.phase.control(:,8);
tau = [ts_e;ts_f;te_e;te_f];
tau_s2 = tau(1)-tau(2)-2*TP(1);
tau_e2 = tau(3)-tau(4)-2*TP(2);

%u2 = input.phase.control(:,5);
%uv2p = input.phase.control(:,6);
%uv2n = input.phase.control(:,7);
%uv2 = uv2p - uv2n;

v_imp2 = input.phase.state(:,1);
i_err2 = input.phase.state(:,2);
err2 = v_imp2/input.auxdata.PAR.Rr-qd2(3);

[M2,ke2,~,H2,k2] = four_bar_system(q2, qd2,input.auxdata.PAR,'Type','Reference');
% l_c2 = Kte*((Jtotal*Ra+La*Btotal)*err2+(Btotal*Ra+Ke)*i_err2+La*Jtotal*(1/input.auxdata.PAR.Td)*(err2-d_err2))/Kmotor;
u2 = Kte*(Jtotal*err2+Btotal*i_err2);

output.phase.RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2 + [0;0;u2];
output.phase.MassMatrix = M2;
output.phase.path = [q2(:,1)-q2(:,2)...
		tau(2)-TA(1), tau(1)+TA(2), ...
        tau(4)-TA(3), tau(3)+TA(4)];

ddx2 = (1/Mi)*(-Ci*v_imp2-Fi);
output.phase.derivatives = [ddx2,err2];

[dalphai,dbetai] = idof_transform(input.phase.initial.position(1),input.phase.initial.position(2),input.phase.initial.position(3),input.phase.initial.velocity(3),input.auxdata.PAR);
[dalphaf,dbetaf] = idof_transform(input.phase.final.position(1),input.phase.final.position(2),input.auxdata.PAR.thetai,input.phase.final.velocity(3),input.auxdata.PAR);

output.constraints = [dalphai-input.phase.initial.velocity(1),dbetai-input.phase.initial.velocity(2), ...
    dalphaf-input.phase.final.velocity(1),dbetaf-input.phase.final.velocity(2)];

obj = integrate(input.phase.integrand,input.phase.control(:,1).^2+input.phase.control(:,2).^2);
obj = obj + integrate(input.phase.integrand,input.phase.control(:,3).^2+input.phase.control(:,4).^2);
output.objective = obj;