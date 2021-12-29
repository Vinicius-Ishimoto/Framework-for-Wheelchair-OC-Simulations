function output = multi_cycle_PID_controller(PAR,ncycle,speed,dist,varargin)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin==1
    error = mesh_error(PAR);
    Mpos = max([error.phase(:).positionmax]);
    Mvel = max([error.phase(:).velocitymax]);
    Merr = max([Mpos,Mvel]);
    output = Merr;
    return
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p = inputParser;
p.StructExpand = false;

addParamValue(p,'Guess',-1);
addParamValue(p,'type','fixed');
addParamValue(p,'Max_control',inf);
addParamValue(p,'Contact_angle',0,@(x) assert(x*pi/180<PAR.thetac2-PAR.thetac1,['Contact angle need to be smaller than ',num2str((PAR.thetac2-PAR.thetac1)*180/pi),' degrees.']));
addParamValue(p,'thetaf',PAR.thetaf*180/pi,@(x) assert(x*pi/180<PAR.thetac2,['Contact angle need to be smaller than ',num2str((PAR.thetac2)*180/pi),' degrees.']));
addParamValue(p,'Max_shoulder',-1,@isnumeric);
addParamValue(p,'Max_elbow',-1,@isnumeric);
addParamValue(p,'ControllerK',PAR.const,@isnumeric);
addParamValue(p,'indice',2,@isnumeric);
addParameter(p,'Mi',PAR.Mi,@isnumeric);
addParamValue(p,'Ci',PAR.Ci,@isnumeric);
addParamValue(p,'Fixed',false,@islogical);
addParamValue(p,'FixedStart',false,@islogical);

parse(p,varargin{:})
if isnumeric(p.Results.Guess)
    opt = multi_cycle_PID_integrate(PAR,opt);
else
    opt = p.Results.Guess;
end
type = p.Results.type;
thetaf = p.Results.thetaf;
MContr = p.Results.Max_control;
isFixed = p.Results.Fixed;
hasFixedStart = p.Results.FixedStart;
MShoulder = p.Results.Max_shoulder;
if MShoulder < 0
	MShoulder = MContr;
end
MElbow = p.Results.Max_elbow;
if MElbow < 0
	MElbow = MContr;
end
Contact = p.Results.Contact_angle*pi/180;

setup.auxdata.PAR = PAR;
setup.auxdata.Td = PAR.Td;
setup.auxdata.const = p.Results.ControllerK;
setup.auxdata.ncycle = ncycle;
setup.auxdata.PAR.Mi = p.Results.Mi;
setup.auxdata.PAR.Ci = p.Results.Ci;
setup.auxdata.indice = p.Results.indice;
% setup.auxdata.alphai  = alphai;
% setup.auxdata.betai  = betai;
% setup.mesh_points = 5;
% setup.mesh_number = 10;
% setup.function = @Dynamics_Multi;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% setup.auxdata.iJi = [-(PAR.R*sin(betai - PAR.thetai))/(PAR.A*sin(alphai - betai)),(PAR.R*sin(alphai - PAR.thetai))/(PAR.B*sin(alphai - betai)),1];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%% BOUNDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cont = 1;
ub = [];
lb = [];
if strcmp(type,'free')
    setup.bound.lower.parameter = [1,0.01];
    setup.bound.upper.parameter = [inf,inf];
    setup.initial_guess.parameter = [PAR.Mi,PAR.Ci];
    setup.function = @Dynamics_Multi_free;
else
    setup.function = @Dynamics_Multi;
end
for q = 1:ncycle
	if cont==1
		setup.bound.lower.phase(cont).initial.velocity = 0;
		setup.bound.upper.phase(cont).initial.velocity = 0;
		setup.bound.lower.phase(cont).initial.state = [0,0,0,0];
		setup.bound.upper.phase(cont).initial.state = [0,0,0,0];
		setup.bound.lower.phase(cont).initial.time = 0;
		setup.bound.upper.phase(cont).initial.time = 0;
	else
		setup.bound.lower.phase(cont).initial.velocity = 0;
		setup.bound.upper.phase(cont).initial.velocity = 50;
		setup.bound.lower.phase(cont).initial.state = [0,-inf,-inf,-inf];
		setup.bound.upper.phase(cont).initial.state = [inf,inf,inf,inf];
		setup.bound.lower.phase(cont).initial.time = 0;
		setup.bound.upper.phase(cont).initial.time = inf;
	end
	if not(isFixed)
		setup.bound.lower.phase(cont).initial.position = PAR.thetac1;
		setup.bound.lower.phase(cont).final.position = PAR.thetac1;
		setup.bound.upper.phase(cont).initial.position = PAR.thetac2;
		setup.bound.upper.phase(cont).final.position = PAR.thetac2;    
	
	else
		setup.bound.lower.phase(cont).initial.position = PAR.thetai;
		setup.bound.lower.phase(cont).final.position = PAR.thetaf;
		setup.bound.upper.phase(cont).initial.position = PAR.thetai;
		setup.bound.upper.phase(cont).final.position = PAR.thetaf;
	end
	setup.bound.lower.phase(cont).final.velocity = 0;
	setup.bound.lower.phase(cont).final.state = [0,-inf,-inf,-inf];
	setup.bound.lower.phase(cont).final.time = 0.1;
	setup.bound.lower.phase(cont).position = PAR.thetac1;
	setup.bound.lower.phase(cont).velocity = 0;
	setup.bound.lower.phase(cont).state = [0,-inf,-inf,-inf];
	setup.bound.lower.phase(cont).control = [0,0,0,-pi/2,0,0,-inf];
	setup.bound.lower.phase(cont).path = [0,0,0,0];
	setup.phase(cont).mesh_points = 8;
	setup.phase(cont).mesh_number = 8;
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	setup.bound.upper.phase(cont).final.velocity = 50;
	setup.bound.upper.phase(cont).final.state = [inf,inf,inf,inf];
	setup.bound.upper.phase(cont).final.time = inf;
	setup.bound.upper.phase(cont).position = PAR.thetac2;
	setup.bound.upper.phase(cont).velocity = 50;
	setup.bound.upper.phase(cont).state = [inf,inf,inf,inf];
	setup.bound.upper.phase(cont).control = [MShoulder,MElbow,pi,pi,MShoulder,MElbow,inf];
	setup.bound.upper.phase(cont).path = [pi,0,0,0];
	cont = cont + 1;
	
	setup.bound.lower.phase(cont).initial.position = [0,-pi/2,PAR.thetac1];
	setup.bound.lower.phase(cont).initial.velocity = [-10*pi,-10*pi,0];
	setup.bound.lower.phase(cont).initial.state = [0,-inf,-inf,-inf];
	setup.bound.lower.phase(cont).initial.time = 0.1;
	% setup.bound.lower.phase(cont).final.position = [0,-pi/2,-150*pi/180];
	setup.bound.lower.phase(cont).final.position = [-inf(1,2),PAR.thetac1];
	setup.bound.lower.phase(cont).final.velocity = [-10*pi,-10*pi,0];
	setup.bound.lower.phase(cont).final.state = [0,-inf,-inf,-inf];
	setup.bound.lower.phase(cont).final.time = 0.2;
	setup.bound.lower.phase(cont).position = [0,-pi/2,PAR.thetac1];
	setup.bound.lower.phase(cont).velocity = [-10*pi,-10*pi,0];
	setup.bound.lower.phase(cont).state = [0,-inf,-inf,-inf];
	setup.bound.lower.phase(cont).control = [0,0,0,0];
	setup.bound.lower.phase(cont).path = 0;
	setup.phase(cont).mesh_points = 10;
	setup.phase(cont).mesh_number = 8;
	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	setup.bound.upper.phase(cont).initial.position = [pi,pi,PAR.thetac2];
	setup.bound.upper.phase(cont).initial.velocity = [10*pi,10*pi,50];
	setup.bound.upper.phase(cont).initial.state = [inf,inf,inf,inf];
	setup.bound.upper.phase(cont).initial.time = inf;
	% setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
	setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
	setup.bound.upper.phase(cont).final.velocity = [10*pi,10*pi,50];
	setup.bound.upper.phase(cont).final.state = [inf,inf,inf,inf];
	setup.bound.upper.phase(cont).final.time = inf;
	setup.bound.upper.phase(cont).position = [pi,pi,inf];
	setup.bound.upper.phase(cont).velocity = [10*pi,10*pi,50];
	setup.bound.upper.phase(cont).state = [inf,inf,inf,inf];
	setup.bound.upper.phase(cont).control = [MShoulder,MElbow,MShoulder,MElbow];
	setup.bound.upper.phase(cont).path = pi;
	if hasFixedStart
		ub = [ub,zeros(1,13),  inf,  inf,zeros(1,8),    inf,0];
		lb = [lb,zeros(1,13),0.001,0.001,zeros(1,8),Contact,0];
	else
		ub = [ub,zeros(1,13),  inf,  inf,zeros(1,8),    inf,inf];
		lb = [lb,zeros(1,13),0.001,0.001,zeros(1,8),Contact,-inf];
	end
	cont = cont + 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.lower.phase(cont).initial.position = PAR.thetac1;
setup.bound.lower.phase(cont).initial.velocity = 0;
setup.bound.lower.phase(cont).initial.state = [0,-inf,-inf,-inf];
setup.bound.lower.phase(cont).initial.time = 0;
setup.bound.lower.phase(cont).final.position = PAR.thetac1;
setup.bound.lower.phase(cont).final.velocity = 0;
setup.bound.lower.phase(cont).final.state = [0,-inf,-inf,-inf];
setup.bound.lower.phase(cont).final.time = dist/speed;
setup.bound.lower.phase(cont).position = PAR.thetac1;
setup.bound.lower.phase(cont).velocity = 0;
setup.bound.lower.phase(cont).state = [-inf,-inf,-inf,-inf];
setup.bound.lower.phase(cont).control = [0,0,0,-pi/2,0,0,-inf];
setup.bound.lower.phase(cont).path = [0,0,0,0];
setup.phase(cont).mesh_points = 5;
setup.phase(cont).mesh_number = 8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.upper.phase(cont).initial.position = PAR.thetac2;
setup.bound.upper.phase(cont).initial.velocity = 50;
setup.bound.upper.phase(cont).initial.state = [inf,inf,inf,inf];
setup.bound.upper.phase(cont).initial.time = inf;
setup.bound.upper.phase(cont).final.position = PAR.thetac2;
setup.bound.upper.phase(cont).final.velocity = inf;
setup.bound.upper.phase(cont).final.state = [inf,inf,inf,inf];
setup.bound.upper.phase(cont).final.time = dist/speed;
setup.bound.upper.phase(cont).position = PAR.thetac2;
setup.bound.upper.phase(cont).velocity = 50;
setup.bound.upper.phase(cont).state = [inf,inf,inf,inf];
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,pi,pi,MShoulder,MElbow,inf];
setup.bound.upper.phase(cont).path = [pi,0,0,0];
if hasFixedStart
	setup.bound.lower.pconstraints = [lb,0.001,dist/PAR.Rr,  1,Contact,0];
	setup.bound.upper.pconstraints = [ub,  inf,dist/PAR.Rr,inf,inf,0];
else
	setup.bound.lower.pconstraints = [lb,0.001,dist/PAR.Rr,  1,Contact,-inf];
	setup.bound.upper.pconstraints = [ub,  inf,dist/PAR.Rr,inf,inf,inf];
end
setup.bound.lower.parameter = [PAR.thetac1];
setup.bound.upper.parameter = [PAR.thetac2];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL GUESS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if not(isfield(opt.solution.phase(1),'state'))
	cont = 1;
	for q = 1:ncycle
		setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
		setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
		setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
		setup.initial_guess.phase(cont).state = [opt.solution.phase(cont).velocity*PAR.Rr,zeros(numel(opt.solution.phase(cont).velocity),3)];
		setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control(:,1:2),opt.solution.phase(cont).control(:,3:4), ...
													opt.solution.phase(cont).control(:,5:6),opt.solution.phase(cont).control(:,7)];
		cont = cont + 1;
		
		setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
		setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
		setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
		setup.initial_guess.phase(cont).state = [opt.solution.phase(cont).velocity*PAR.Rr,zeros(numel(opt.solution.phase(cont).velocity),3)];
		setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control(:,1:2),opt.solution.phase(cont).control(:,3:4)];
		cont = cont + 1;
	end
	setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
	setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
	setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
	setup.initial_guess.phase(cont).state = [opt.solution.phase(1).velocity*PAR.Rr,zeros(numel(opt.solution.phase(1).velocity),3)];
	setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control(:,1:2),opt.solution.phase(cont).control(:,3:4), ...
												opt.solution.phase(cont).control(:,5:6),opt.solution.phase(cont).control(:,7)];
	setup.initial_guess.parameter = -100*pi/180;
	output = ddiopt_MB(setup);
else
	cont = 1;
	for q = 1:ncycle
		setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
		setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
		setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
		setup.initial_guess.phase(cont).state = opt.solution.phase(cont).state;
		setup.initial_guess.phase(cont).control = opt.solution.phase(cont).control;
		cont = cont + 1;
		
		setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
		setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
		setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
		setup.initial_guess.phase(cont).state = opt.solution.phase(cont).state;
		setup.initial_guess.phase(cont).control = opt.solution.phase(cont).control;
		cont = cont + 1;
	end
	setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
	setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
	setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
	setup.initial_guess.phase(cont).state = opt.solution.phase(cont).state;
	setup.initial_guess.phase(cont).control = opt.solution.phase(cont).control;
	setup.initial_guess.parameter = -100*pi/180;
    output = ddiopt_MB(setup);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cont = 1;
for q = 1:ncycle
    output.solution.phase(cont).taup = output.solution.phase(cont).control(:,7);
    cont = cont + 1;
    
    output.solution.phase(cont).taup = zeros(numel(output.solution.phase(cont).velocity(:,1)),1);
    cont = cont + 1;
end
output.solution.phase(cont).taup = output.solution.phase(cont).control(:,7);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Options used in this simulation
output.Options.NumCycles = ncycle;
output.Options.TotalDistance = dist;
output.Options.MeanVelocity = speed;
output.Options.Guess = setup.initial_guess;
output.Options.MinContactAngle = Contact;
output.Options.thetaf = thetaf;
output.Options.MaxControl = MContr;
output.Options.MaxShoulder = MShoulder;
output.Options.MaxElbow = MElbow;
output.Options.Indice = setup.auxdata.indice;
output.Options.FixedContactAngle = isFixed;
output.Options.FixedStartAngle = hasFixedStart;
output.Options.KController = p.Results.ControllerK;
output.Options.ImpedanceMass = p.Results.Mi;
output.Options.ImpedanceFriction = p.Results.Ci;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function output = Dynamics_Multi(input)

Mi = input.auxdata.PAR.Mi;
Ci = input.auxdata.PAR.Ci;
Kte = input.auxdata.const;
Jtotal = input.auxdata.PAR.Jmt;
Btotal = input.auxdata.PAR.Bt;
Kmotor = input.auxdata.PAR.rt*input.auxdata.PAR.Kt;
Ke = input.auxdata.PAR.rt*input.auxdata.PAR.Ke;
Ra = input.auxdata.PAR.Ra;
La = input.auxdata.PAR.La;
Td = input.auxdata.PAR.Td;
indice = input.auxdata.indice;

p_constraints = [];
obj = 0;
theta_total = 0;
cont = 1;
for q=1:input.auxdata.ncycle
theta1 = input.phase(cont).position;
dtheta1 = input.phase(cont).velocity;
tau_s1 = input.phase(cont).control(:,1)-input.phase(cont).control(:,5);
tau_e1 = input.phase(cont).control(:,2)-input.phase(cont).control(:,6);
alpha1 = input.phase(cont).control(:,3);
beta1 = input.phase(cont).control(:,4);
u1 = input.phase(cont).control(:,7);
v_imp1 = input.phase(cont).state(:,1);
i_err1 = input.phase(cont).state(:,2);
i_motor1 = input.phase(cont).state(:,3);
d_err1 = input.phase(cont).state(:,4);
theta_initial = col(input.phase(cont).parameter,1);

err1 = v_imp1/input.auxdata.PAR.Rr-dtheta1;
[dalpha1,dbeta1,PHI,dPHI] = idof_transform(alpha1,beta1,theta1,dtheta1,input.auxdata.PAR);
q1 = [alpha1,beta1,theta1];
qd1 = [dalpha1,dbeta1,dtheta1];

[M1,ke1,~,H1,k1] = four_bar_system(q1, qd1,input.auxdata.PAR);
l_c1 = Kte*((Jtotal*Ra+La*Btotal)*err1+(Btotal*Ra+Ke)*i_err1+La*Jtotal*(1/input.auxdata.Td)*(err1-d_err1))/Kmotor;
K = PHI'*M1*dPHI*dtheta1;
% tau_p1 = 100*exp(-((alpha1-beta1)/0.2).^2);
output.phase(cont).RightHandSide = PHI'*(H1*[tau_s1;tau_e1]+ke1-k1)-K+Kmotor*i_motor1;
output.phase(cont).MassMatrix = PHI'*M1*PHI;
ddtheta1 = output.phase(cont).RightHandSide./output.phase(cont).MassMatrix;
qdd1 = dPHI*dtheta1+PHI*ddtheta1;
tp1 = M1(3,:)*qdd1-ke1(3)+k1(3)-Kmotor*i_motor1;
% ddx1 = 1/Mi*(u1-Ci*dx1);
% derr1 = ddx1/input.auxdata.PAR.Rr-ddtheta1;
% dFc1 = Kte*(M1(end)*derr1+input.auxdata.PAR.dFric*input.auxdata.PAR.Rr^2*err1)/input.auxdata.PAR.Kt;
output.phase(cont).derivatives = [(1/Mi)*(u1/input.auxdata.PAR.R-Ci*v_imp1),err1, ...
    (1/La)*(l_c1-Ke*dtheta1-Ra*i_motor1), (1/input.auxdata.Td)*(err1-d_err1)];
output.phase(cont).path = [q1(:,1)-q1(:,2), ...
    input.auxdata.PAR.A*sin(alpha1)+input.auxdata.PAR.B*sin(beta1)-input.auxdata.PAR.R*sin(theta1)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha1)+input.auxdata.PAR.B*cos(beta1)-input.auxdata.PAR.R*cos(theta1)-input.auxdata.PAR.h, ...
    u1-tp1];
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice);
cont = cont + 1;

q2 = input.phase(cont).position;
qd2 = input.phase(cont).velocity;
tau_s2 = input.phase(cont).control(:,1)-input.phase(cont).control(:,3);
tau_e2 = input.phase(cont).control(:,2)-input.phase(cont).control(:,4);
v_imp2 = input.phase(cont).state(:,1);
i_err2 = input.phase(cont).state(:,2);
d_err2 = input.phase(cont).state(:,4);
i_motor2 = input.phase(cont).state(:,3);
err2 = v_imp2/input.auxdata.PAR.Rr-qd2(3);
% tau_p2 = 100*exp(-((q2(:,1)-q2(:,2))/0.2).^2);

[M2,ke2,~,H2,k2] = four_bar_system(q2, qd2,input.auxdata.PAR);
l_c2 = Kte*((Jtotal*Ra+La*Btotal)*err2+(Btotal*Ra+Ke)*i_err2+La*Jtotal*(1/input.auxdata.Td)*(err2-d_err2))/Kmotor;
output.phase(cont).RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2 + [0;0;Kmotor*i_motor2];
output.phase(cont).MassMatrix = M2;
% qdd2 = output.phase(cont).MassMatrix\output.phase(cont).RightHandSide;
output.phase(cont).path = q2(:,1)-q2(:,2);

ddx2 = (1/Mi)*(-Ci*v_imp2);
% derr2 = ddx2/input.auxdata.PAR.Rr-qdd2(3);
% dFc2 = Kte*(M2(end)*derr2+input.auxdata.PAR.dFric*input.auxdata.PAR.Rr^2*err2)/input.auxdata.PAR.Kt;
output.phase(cont).derivatives = [ddx2,err2, ...
    (1/La)*(l_c2-Ke*qd2(3)-Ra*i_motor2), (1/input.auxdata.Td)*(err2-d_err2)];

[dalphai,dbetai,~,~] = idof_transform(input.phase(cont).initial.position(1),input.phase(cont).initial.position(2),input.phase(cont).initial.position(3),input.phase(cont).initial.velocity(3),input.auxdata.PAR);
[dalphaf,dbetaf,PHIf,~] = idof_transform(input.phase(cont).final.position(1),input.phase(cont).final.position(2),input.phase(cont+1).initial.position,input.phase(cont).final.velocity(3),input.auxdata.PAR);
p_constraints = [p_constraints, input.phase(cont).initial.time-input.phase(cont-1).final.time];
p_constraints = [p_constraints,input.phase(cont).initial.position(3)-input.phase(cont-1).final.position, ...
    input.auxdata.PAR.A*sin(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(cont).initial.position(3))-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(cont).initial.position(3))-input.auxdata.PAR.h, ...
    dalphai-input.phase(cont).initial.velocity(1),dbetai-input.phase(cont).initial.velocity(2), ...
    input.phase(cont).initial.velocity(3)-input.phase(cont-1).final.velocity, ...
    input.auxdata.PAR.A*sin(input.phase(cont).final.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).final.position(2))-input.auxdata.PAR.R*sin(input.phase(cont+1).initial.position)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).final.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).final.position(2))-input.auxdata.PAR.R*cos(input.phase(cont+1).initial.position)-input.auxdata.PAR.h, ...
    input.phase(cont).final.velocity(1)-dalphaf,input.phase(cont).final.velocity(2)-dbetaf,input.phase(cont).final.velocity(3)-input.phase(cont+1).initial.velocity, ...
    input.phase(cont+1).initial.time-input.phase(cont).final.time, ...
    input.phase(cont-1).final.time-input.phase(cont-1).initial.time, ...
    input.phase(cont).final.time-input.phase(cont).initial.time, ...
	theta_initial - input.phase(cont-1).initial.position];

p_constraints = [p_constraints,input.phase(cont-1).final.state-input.phase(cont).initial.state, ...
    input.phase(cont).final.state-input.phase(cont+1).initial.state, ...
	input.phase(cont-1).final.position-input.phase(cont-1).initial.position];

obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,3).^indice+input.phase(cont).control(:,4).^indice);
theta_total = theta_total + input.phase(cont).final.position(3)-input.phase(cont-1).initial.position(1);
cont = cont + 1;
end
theta3 = input.phase(cont).position;
dtheta3 = input.phase(cont).velocity;
tau_s3 = input.phase(cont).control(:,1)-input.phase(cont).control(:,5);
tau_e3 = input.phase(cont).control(:,2)-input.phase(cont).control(:,6);
alpha3 = input.phase(cont).control(:,3);
beta3 = input.phase(cont).control(:,4);
u3 = input.phase(cont).control(:,7);
v_imp3 = input.phase(cont).state(:,1);
i_err3 = input.phase(cont).state(:,2);
d_err3 = input.phase(cont).state(:,4);
i_motor3 = input.phase(cont).state(:,3);
err3 = v_imp3/input.auxdata.PAR.Rr-dtheta3;
theta_initial = col(input.phase(cont).parameter,1);

[dalpha3,dbeta3,PHI,dPHI] = idof_transform(alpha3,beta3,theta3,dtheta3,input.auxdata.PAR);
q3 = [alpha3,beta3,theta3];
qd3 = [dalpha3,dbeta3,dtheta3];

[M3,ke3,~,H3,k3] = four_bar_system(q3, qd3,input.auxdata.PAR);
l_c3 = Kte*((Jtotal*Ra+La*Btotal)*err3+(Btotal*Ra+Ke)*i_err3+La*Jtotal*(1/input.auxdata.Td)*(err3-d_err3))/Kmotor;
K = PHI'*M3*dPHI*dtheta3;
% tau_p3 = 100*exp(-((alpha3-beta3)/0.2).^2);
output.phase(cont).RightHandSide = PHI'*(H3*[tau_s3;tau_e3]+ke3-k3)-K+Kmotor*i_motor3;
output.phase(cont).MassMatrix = PHI'*M3*PHI;

ddtheta3 = output.phase(cont).RightHandSide./output.phase(cont).MassMatrix;
qdd3 = dPHI*dtheta3+PHI*ddtheta3;
dv_imp3 = 1/Mi*(u3/input.auxdata.PAR.R-Ci*v_imp3);
% derr1 = ddx1/input.auxdata.PAR.Rr-ddtheta1;
% dFc1 = Kte*(M1(end)*derr1+input.auxdata.PAR.dFric*input.auxdata.PAR.Rr^2*err1)/input.auxdata.PAR.Kt;
tp3 = M3(3,:)*qdd3-ke3(3)+k3(3)-Kmotor*i_motor3;

obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice);

output.phase(cont).derivatives = [dv_imp3,err3, ...
    (1/La)*(l_c3-Ke*dtheta3-Ra*i_motor3), (1/input.auxdata.Td)*(err3-d_err3)];

output.phase(cont).path = [q3(:,1)-q3(:,2), ...
    input.auxdata.PAR.A*sin(alpha3)+input.auxdata.PAR.B*sin(beta3)-input.auxdata.PAR.R*sin(theta3)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha3)+input.auxdata.PAR.B*cos(beta3)-input.auxdata.PAR.R*cos(theta3)-input.auxdata.PAR.h, ...
    tp3-u3];

p_constraints = [p_constraints, input.phase(cont).final.time-input.phase(cont).initial.time, ...
    theta_total+input.phase(cont).final.position-input.phase(cont).initial.position, obj, ...
	input.phase(cont).final.position-input.phase(cont).initial.position, ...
	theta_initial - input.phase(cont).initial.position];

%      ...

output.constraints = p_constraints;
% output.constraints = [input.phase(2).initial.time-input.phase(1).final.time,input.phase(2).final.time-input.phase(2).initial.time, ...
%     input.phase(1).final.position(3)-input.phase(1).final.position(2),input.phase(2).final.position(3)-input.phase(2).final.position(2)];
output.objective = obj;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function output = Dynamics_Multi_free(input)

Kte = input.auxdata.const;
Jtotal = input.auxdata.PAR.Jmt;
Btotal = input.auxdata.PAR.Bt;
% delay = input.auxdata.PAR.delay;
Kmotor = input.auxdata.PAR.rt*input.auxdata.PAR.Kt;
Ke = input.auxdata.PAR.rt*input.auxdata.PAR.Ke;
Ra = input.auxdata.PAR.Ra;
La = input.auxdata.PAR.La;

p_constraints = [];
obj = 0;
theta_total = 0;
cont = 1;
for q=1:input.auxdata.ncycle
theta1 = input.phase(cont).position;
dtheta1 = input.phase(cont).velocity;
tau_s1 = input.phase(cont).control(:,1);
tau_e1 = input.phase(cont).control(:,2);
alpha1 = input.phase(cont).control(:,3);
beta1 = input.phase(cont).control(:,4);
u1 = input.phase(cont).control(:,5);
v_imp1 = input.phase(cont).state(:,1);
i_err1 = input.phase(cont).state(:,2);
i_motor1 = input.phase(cont).state(:,3);
d_err1 = input.phase(cont).state(:,4);
Mi = input.phase(cont).parameter(1);
Ci = input.phase(cont).parameter(2);

err1 = v_imp1/input.auxdata.PAR.Rr-dtheta1;
[dalpha1,dbeta1,PHI,dPHI] = idof_transform(alpha1,beta1,theta1,dtheta1,input.auxdata.PAR);
q1 = [alpha1,beta1,theta1];
qd1 = [dalpha1,dbeta1,dtheta1];

[M1,ke1,~,H1,k1] = four_bar_system(q1, qd1,input.auxdata.PAR);
l_c1 = Kte*((Jtotal*Ra+La*Btotal)*err1+(Btotal*Ra+Ke)*i_err1+La*Jtotal*(1/input.auxdata.Td)*(err1-d_err1))/Kmotor;
K = PHI'*M1*dPHI*dtheta1;
% tau_p1 = 100*exp(-((alpha1-beta1)/0.2).^2);
output.phase(cont).RightHandSide = PHI'*(H1*[tau_s1;tau_e1]+ke1-k1)-K+Kmotor*i_motor1;
output.phase(cont).MassMatrix = PHI'*M1*PHI;
ddtheta1 = output.phase(cont).RightHandSide./output.phase(cont).MassMatrix;
qdd1 = dPHI*dtheta1+PHI*ddtheta1;
tp1 = M1(3,:)*qdd1-ke1(3)+k1(3)-Kmotor*i_motor1;
% ddx1 = 1/Mi*(u1-Ci*dx1);
% derr1 = ddx1/input.auxdata.PAR.Rr-ddtheta1;
% dFc1 = Kte*(M1(end)*derr1+input.auxdata.PAR.dFric*input.auxdata.PAR.Rr^2*err1)/input.auxdata.PAR.Kt;
output.phase(cont).derivatives = [(1/Mi)*(u1/input.auxdata.PAR.R-Ci*v_imp1),err1, ...
    (1/La)*(l_c1-Ke*dtheta1-Ra*i_motor1), (1/input.auxdata.Td)*(err1-d_err1)];
output.phase(cont).path = [q1(:,1)-q1(:,2), ...
    input.auxdata.PAR.A*sin(alpha1)+input.auxdata.PAR.B*sin(beta1)-input.auxdata.PAR.R*sin(theta1)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha1)+input.auxdata.PAR.B*cos(beta1)-input.auxdata.PAR.R*cos(theta1)-input.auxdata.PAR.h, ...
    u1-tp1];
obj = obj + integrate(input.phase(cont).integrand,tau_s1.^2+tau_e1.^2) + integrate(input.phase(cont).integrand,(Kmotor*i_motor1).^2);
cont = cont + 1;

q2 = input.phase(cont).position;
qd2 = input.phase(cont).velocity;
tau_s2 = input.phase(cont).control(:,1);
tau_e2 = input.phase(cont).control(:,2);
v_imp2 = input.phase(cont).state(:,1);
i_err2 = input.phase(cont).state(:,2);
d_err2 = input.phase(cont).state(:,4);
i_motor2 = input.phase(cont).state(:,3);
err2 = v_imp2/input.auxdata.PAR.Rr-qd2(3);
Mi = input.phase(cont).parameter(1);
Ci = input.phase(cont).parameter(2);
% tau_p2 = 100*exp(-((q2(:,1)-q2(:,2))/0.2).^2);

[M2,ke2,~,H2,k2] = four_bar_system(q2, qd2,input.auxdata.PAR);
l_c2 = Kte*((Jtotal*Ra+La*Btotal)*err2+(Btotal*Ra+Ke)*i_err2+La*Jtotal*(1/input.auxdata.Td)*(err2-d_err2))/Kmotor;
output.phase(cont).RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2 + [0;0;Kmotor*i_motor2];
output.phase(cont).MassMatrix = M2;
% qdd2 = output.phase(cont).MassMatrix\output.phase(cont).RightHandSide;
output.phase(cont).path = q2(:,1)-q2(:,2);

ddx2 = (1/Mi)*(-Ci*v_imp2);
% derr2 = ddx2/input.auxdata.PAR.Rr-qdd2(3);
% dFc2 = Kte*(M2(end)*derr2+input.auxdata.PAR.dFric*input.auxdata.PAR.Rr^2*err2)/input.auxdata.PAR.Kt;
output.phase(cont).derivatives = [ddx2,err2, ...
    (1/La)*(l_c2-Ke*qd2(3)-Ra*i_motor2), (1/input.auxdata.Td)*(err2-d_err2)];

[dalphai,dbetai,~,~] = idof_transform(input.phase(cont).initial.position(1),input.phase(cont).initial.position(2),input.phase(cont).initial.position(3),input.phase(cont).initial.velocity(3),input.auxdata.PAR);
[dalphaf,dbetaf,PHIf,~] = idof_transform(input.phase(cont).final.position(1),input.phase(cont).final.position(2),input.phase(cont+1).initial.position,input.phase(cont).final.velocity(3),input.auxdata.PAR);
p_constraints = [p_constraints, input.phase(cont).initial.time-input.phase(cont-1).final.time];
p_constraints = [p_constraints,input.phase(cont).initial.position(3)-input.phase(cont-1).final.position, ...
    input.auxdata.PAR.A*sin(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(cont).initial.position(3))-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(cont).initial.position(3))-input.auxdata.PAR.h, ...
    dalphai-input.phase(cont).initial.velocity(1),dbetai-input.phase(cont).initial.velocity(2), ...
    input.phase(cont).initial.velocity(3)-input.phase(cont-1).final.velocity, ...
    input.auxdata.PAR.A*sin(input.phase(cont).final.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).final.position(2))-input.auxdata.PAR.R*sin(input.phase(cont+1).initial.position)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).final.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).final.position(2))-input.auxdata.PAR.R*cos(input.phase(cont+1).initial.position)-input.auxdata.PAR.h, ...
    input.phase(cont).final.velocity(1)-dalphaf,input.phase(cont).final.velocity(2)-dbetaf,input.phase(cont).final.velocity(3)-input.phase(cont+1).initial.velocity, ...
    input.phase(cont+1).initial.time-input.phase(cont).final.time, ...
    input.phase(cont-1).final.time-input.phase(cont-1).initial.time, ...
    input.phase(cont).final.time-input.phase(cont).initial.time];

p_constraints = [p_constraints,input.phase(cont-1).final.state-input.phase(cont).initial.state, ...
    input.phase(cont).final.state-input.phase(cont+1).initial.state];

obj = obj + integrate(input.phase(cont).integrand,tau_s2.^2+tau_e2.^2) + integrate(input.phase(cont).integrand,(Kmotor*i_motor2).^2);
theta_total = theta_total + input.phase(cont).final.position(3)-input.phase(cont-1).initial.position(1);
cont = cont + 1;
end
theta3 = input.phase(cont).position;
dtheta3 = input.phase(cont).velocity;
tau_s3 = input.phase(cont).control(:,1);
tau_e3 = input.phase(cont).control(:,2);
alpha3 = input.phase(cont).control(:,3);
beta3 = input.phase(cont).control(:,4);
u3 = input.phase(cont).control(:,5);
v_imp3 = input.phase(cont).state(:,1);
i_err3 = input.phase(cont).state(:,2);
d_err3 = input.phase(cont).state(:,4);
i_motor3 = input.phase(cont).state(:,3);
err3 = v_imp3/input.auxdata.PAR.Rr-dtheta3;
Mi = input.phase(cont).parameter(1);
Ci = input.phase(cont).parameter(2);

[dalpha3,dbeta3,PHI,dPHI] = idof_transform(alpha3,beta3,theta3,dtheta3,input.auxdata.PAR);
q3 = [alpha3,beta3,theta3];
qd3 = [dalpha3,dbeta3,dtheta3];

[M3,ke3,~,H3,k3] = four_bar_system(q3, qd3,input.auxdata.PAR);
l_c3 = Kte*((Jtotal*Ra+La*Btotal)*err3+(Btotal*Ra+Ke)*i_err3+La*Jtotal*(1/input.auxdata.Td)*(err3-d_err3))/Kmotor;
K = PHI'*M3*dPHI*dtheta3;
% tau_p3 = 100*exp(-((alpha3-beta3)/0.2).^2);
output.phase(cont).RightHandSide = PHI'*(H3*[tau_s3;tau_e3]+ke3-k3)-K+Kmotor*i_motor3;
output.phase(cont).MassMatrix = PHI'*M3*PHI;

ddtheta3 = output.phase(cont).RightHandSide./output.phase(cont).MassMatrix;
qdd3 = dPHI*dtheta3+PHI*ddtheta3;
dv_imp3 = 1/Mi*(u3/input.auxdata.PAR.Rr-Ci*v_imp3);
% derr1 = ddx1/input.auxdata.PAR.Rr-ddtheta1;
% dFc1 = Kte*(M1(end)*derr1+input.auxdata.PAR.dFric*input.auxdata.PAR.Rr^2*err1)/input.auxdata.PAR.Kt;
tp3 = M3(3,:)*qdd3-ke3(3)+k3(3)-Kmotor*i_motor3;

output.phase(cont).derivatives = [dv_imp3,err3, ...
    (1/La)*(l_c3-Ke*dtheta3-Ra*i_motor3), (1/input.auxdata.Td)*(err3-d_err3)];

output.phase(cont).path = [q3(:,1)-q3(:,2), ...
    input.auxdata.PAR.A*sin(alpha3)+input.auxdata.PAR.B*sin(beta3)-input.auxdata.PAR.R*sin(theta3)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha3)+input.auxdata.PAR.B*cos(beta3)-input.auxdata.PAR.R*cos(theta3)-input.auxdata.PAR.h, ...
    tp3-u3];

p_constraints = [p_constraints, input.phase(cont).final.time-input.phase(cont).initial.time, ...
    theta_total+input.phase(cont).final.position-input.phase(cont).initial.position,obj + integrate(input.phase(cont).integrand,tau_s3.^2+tau_e3.^2)];

%      ...

output.constraints = p_constraints;
% output.constraints = [input.phase(2).initial.time-input.phase(1).final.time,input.phase(2).final.time-input.phase(2).initial.time, ...
%     input.phase(1).final.position(3)-input.phase(1).final.position(2),input.phase(2).final.position(3)-input.phase(2).final.position(2)];
output.objective = obj + integrate(input.phase(cont).integrand,tau_s3.^2+tau_e3.^2) + integrate(input.phase(cont).integrand,(Kmotor*i_motor3).^2);