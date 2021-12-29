function output = steady_state_MBF_controller(PAR,speed,varargin)
% function output = steady_state_PID_controller(PAR,velocity,opt)

p = inputParser;

addParameter(p,'Guess',-1);
addParameter(p,'Contact_angle',0,@(x) assert(x*pi/180<PAR.thetac2-PAR.thetac1,['Contact angle need to be smaller than ',num2str((PAR.thetac2-PAR.thetac1)*180/pi),' degrees.']));
addParameter(p,'thetaf',PAR.thetaf*180/pi,@(x) assert(x*pi/180<PAR.thetac2,['Contact angle need to be smaller than ',num2str((PAR.thetac2)*180/pi),' degrees.']));
addParameter(p,'Max_control',inf,@isnumeric);
addParameter(p,'Max_shoulder',-1,@isnumeric);
addParameter(p,'Max_elbow',-1,@isnumeric);
addParameter(p,'ControllerK',PAR.const,@isnumeric);
addParameter(p,'indice',2,@isnumeric);
addParameter(p,'Mi',PAR.Mi,@isnumeric);
addParameter(p,'Ci',PAR.Ci,@isnumeric);
addParameter(p,'MinMax',false,@islogical);
addParameter(p,'Fixed',false,@islogical);
addParameter(p,'FixedStart',false,@islogical);
parse(p,varargin{:});
optr = p.Results.Guess;
Contact = p.Results.Contact_angle*pi/180;
MContr = p.Results.Max_control;
MShoulder = p.Results.Max_shoulder;
MCurr = inf;
if MShoulder < 0
	MShoulder = MContr;
end
MElbow = p.Results.Max_elbow;
if MElbow < 0
	MElbow = MContr;
end
isFixed = p.Results.Fixed;
hasFixedStart = p.Results.FixedStart;

setup.auxdata.Td = PAR.Td;
setup.auxdata.const = p.Results.ControllerK;
setup.auxdata.PAR = PAR;
setup.auxdata.PAR.Mi = p.Results.Mi;
setup.auxdata.PAR.Ci = p.Results.Ci;
% setup.auxdata.ncycle = ncycle;
setup.auxdata.speed = speed;
setup.auxdata.indice = p.Results.indice;
% setup.auxdata.alphai  = alphai;
% setup.auxdata.betai  = betai;
% setup.auxdata.iJi = [-(PAR.R*sin(betai - PAR.thetai))/(PAR.A*sin(alphai - betai)),(PAR.R*sin(alphai - PAR.thetai))/(PAR.B*sin(alphai - betai)),1];
setup.function = @Dynamics_Multi;

cont = 1;
if isFixed
    setup.bound.lower.phase(cont).final.position = thetaf;
    setup.bound.upper.phase(cont).final.position = thetaf;
    setup.bound.lower.phase(cont).initial.position = thetai;
    setup.bound.upper.phase(cont).initial.position = thetai;
elseif hasFixedStart
    setup.bound.lower.phase(cont).final.position = PAR.thetac1;
    setup.bound.upper.phase(cont).final.position = PAR.thetac2;
    setup.bound.lower.phase(cont).initial.position = thetai;
    setup.bound.upper.phase(cont).initial.position = thetai;
else
    setup.bound.lower.phase(cont).final.position = PAR.thetac1;
    setup.bound.upper.phase(cont).final.position = PAR.thetac2;
    setup.bound.lower.phase(cont).initial.position = PAR.thetac1;
    setup.bound.upper.phase(cont).initial.position = PAR.thetac2;
end

setup.bound.lower.phase(cont).initial.velocity = 0;
setup.bound.upper.phase(cont).initial.velocity = 50;
setup.bound.lower.phase(cont).initial.state = -inf;
setup.bound.lower.phase(cont).initial.time = 0;
setup.bound.upper.phase(cont).initial.time = 0;
% setup.bound.lower.phase(cont).initial.position = PAR.thetai;
% setup.bound.lower.phase(cont).final.position = PAR.thetac1;
setup.bound.lower.phase(cont).final.velocity = 0;
setup.bound.lower.phase(cont).final.state = -inf;
setup.bound.lower.phase(cont).final.time = 0.1;
setup.bound.lower.phase(cont).position = PAR.thetac1;
setup.bound.lower.phase(cont).velocity = 0;
setup.bound.lower.phase(cont).state = -inf;
setup.bound.lower.phase(cont).control = [0,0,0,-pi/2,0,0,-inf];
setup.bound.lower.phase(cont).path = [0,0,0,0];
setup.phase(cont).mesh_points = 8;
setup.phase(cont).mesh_number = 8;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup.bound.upper.phase(cont).initial.position = PAR.thetai;
setup.bound.upper.phase(cont).initial.state = inf;
% setup.bound.upper.phase(cont).final.position = PAR.thetac2;
setup.bound.upper.phase(cont).final.velocity = 50;
setup.bound.upper.phase(cont).final.state = inf;
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.upper.phase(cont).position = PAR.thetac2;
setup.bound.upper.phase(cont).velocity = 50;
setup.bound.upper.phase(cont).state = inf;
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,pi,pi,MShoulder,MElbow,inf];
setup.bound.upper.phase(cont).path = [pi,0,0,0];
cont = cont + 1;

setup.bound.lower.phase(cont).initial.position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).initial.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).initial.state = -inf;
setup.bound.lower.phase(cont).initial.time = 0.1;
setup.bound.lower.phase(cont).final.position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).final.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).final.state = -inf;
setup.bound.lower.phase(cont).final.time = 0.2;
setup.bound.lower.phase(cont).position = [0,-pi/2,-150*pi/180];
setup.bound.lower.phase(cont).velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).state = -inf;
setup.bound.lower.phase(cont).control = [0,0,0,0];
setup.bound.lower.phase(cont).path = 0;
setup.phase(cont).mesh_points = 12;
setup.phase(cont).mesh_number = 10;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.upper.phase(cont).initial.position = [pi,pi,PAR.thetac2];
setup.bound.upper.phase(cont).initial.velocity = [10*pi,10*pi,50];
setup.bound.upper.phase(cont).initial.state = inf;
setup.bound.upper.phase(cont).initial.time = inf;
% setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
setup.bound.upper.phase(cont).final.velocity = [10*pi,10*pi,50];
setup.bound.upper.phase(cont).final.state = inf;
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.upper.phase(cont).position = [pi,pi,inf];
setup.bound.upper.phase(cont).velocity = [10*pi,10*pi,50];
setup.bound.upper.phase(cont).state = inf;
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,MShoulder,MElbow];
setup.bound.upper.phase(cont).path = pi;

% setup.bound.lower.parameter = [0,0];
% setup.bound.upper.parameter = [0,0];
setup.bound.lower.pconstraints = [zeros(1,14),0.01,0.01,0,Contact];
setup.bound.upper.pconstraints = [zeros(1,14), inf, inf,0,inf];

%% Initial guess
if isnumeric(optr)
	ini = 'wheelchair_ni40_Froll15_nn40_v05.mat';
	load(ini)
	cont = 1;
	setup.initial_guess.phase(cont).time = opt.t1+opt.t2(end);
	setup.initial_guess.phase(cont).position = opt.x1/PAR.Rr+PAR.thetai;
	setup.initial_guess.phase(cont).velocity = opt.xd1/PAR.Rr;
	setup.initial_guess.phase(cont).state = [opt.xd1/PAR.Rr,-opt.tau_s1,-opt.tau_e1,-opt.tau_s1];
	% setup.initial_guess.phase(1).control = [-opt.tau_s1,-opt.tau_e1,-opt.alpha1,-opt.beta1];
	setup.initial_guess.phase(cont).control = [-opt.tau_s1,-opt.tau_e1,pi/2*ones(41,1),0.1*ones(41,2),opt.tau_s1,opt.tau_e1];
	cont = cont + 1;
	
	setup.initial_guess.phase(cont).time = opt.t2;
	setup.initial_guess.phase(cont).position = [pi/2*ones(41,1),0.1*ones(41,1),opt.x2/PAR.Rr+PAR.thetai];
	setup.initial_guess.phase(cont).velocity = [-opt.alphad2,-opt.betad2,opt.xd2/PAR.Rr];
	setup.initial_guess.phase(cont).control = -[opt.tau_s2,opt.tau_e2,-opt.tau_s2,-opt.tau_e2];
	setup.initial_guess.phase(cont).state = [opt.xd2/PAR.Rr,-opt.tau_s2,-opt.tau_e2,-opt.tau_s2];
	setup.initial_guess.parameter = [1,1];
	output = ddiopt_MB(setup);
else
	setup.initial_guess.phase(1).time = optr.solution.phase(1).time;
	setup.initial_guess.phase(1).position = optr.solution.phase(1).position;
	setup.initial_guess.phase(1).velocity = optr.solution.phase(1).velocity;
	if isfield(optr.solution.phase(1),'state')
		setup.initial_guess.phase(1).state = optr.solution.phase(1).state(:,4);
	else
		setup.initial_guess.phase(1).state = zeros(size(optr.solution.phase(1).velocity,1),1);
	end
	setup.initial_guess.phase(1).control = [optr.solution.phase(1).control(:,1:2), optr.solution.phase(1).control(:,3:4), ...
											optr.solution.phase(1).control(:,5:6), optr.solution.phase(1).control(:,7)];
	setup.initial_guess.phase(2).time = optr.solution.phase(2).time;
	setup.initial_guess.phase(2).position = optr.solution.phase(2).position;
	setup.initial_guess.phase(2).velocity = optr.solution.phase(2).velocity;
	setup.initial_guess.phase(2).control = [optr.solution.phase(2).control(:,1:2),optr.solution.phase(2).control(:,3:4)];
	if isfield(optr.solution.phase(2),'state')
		setup.initial_guess.phase(2).state = optr.solution.phase(2).state(:,4);
	else
		setup.initial_guess.phase(2).state = zeros(size(optr.solution.phase(2).velocity,1),1);
	end
% 	setup.initial_guess.parameter =  [1,1];
	output = ddiopt_MB(setup);
end

output.solution.phase(1).taup = output.solution.phase(1).control(:,7);
% output.solution.phase(1).control(:,5) = output.solution.phase(1).control(:,6);
% output.solution.phase(1).control(:,6) = output.solution.phase(1).control(:,7);
output.solution.phase(2).taup = zeros(numel(output.solution.phase(2).control(:,1)),1);

% Options used in this simulation
output.Options.Type = 'MBF';
output.Options.MeanVelocity = speed;
output.Options.Guess = setup.initial_guess;
output.Options.MinContactAngle = Contact;
output.Options.thetaf = p.Results.thetaf;
output.Options.MaxControl = MContr;
output.Options.MaxShoulder = MShoulder;
output.Options.MaxElbow = MElbow;
output.Options.Indice = setup.auxdata.indice;
output.Options.FixedContactAngle = isFixed;
output.Options.FixedStartAngle = hasFixedStart;
output.Options.KController = p.Results.ControllerK;
output.Options.ImpedanceMass = p.Results.Mi;
output.Options.ImpedanceFriction = p.Results.Ci;


function output = Dynamics_Multi(input)

Mi = input.auxdata.PAR.Mi;
Ci = input.auxdata.PAR.Ci;
Fri = input.auxdata.PAR.Fri;
Kte = input.auxdata.const;
Jtotal = input.auxdata.PAR.Jmt;
% Btotal = input.auxdata.PAR.Bt;
% Kmotor = input.auxdata.PAR.rt*input.auxdata.PAR.Kt;
% Ke = input.auxdata.PAR.rt*input.auxdata.PAR.Ke;
% Ra = input.auxdata.PAR.Ra;
% La = input.auxdata.PAR.La;
Td = input.auxdata.PAR.Td;
indice = input.auxdata.indice;

% sl1 = col(input.phase(1).parameter(1),1);
% sl2 = col(input.phase(1).parameter(2),1);

obj = 0;
% fator = 0;
cont = 1;
theta3 = input.phase(cont).position;
dtheta3 = input.phase(cont).velocity;
tau_s3 = input.phase(cont).control(:,1)-input.phase(cont).control(:,5);
tau_e3 = input.phase(cont).control(:,2)-input.phase(cont).control(:,6);
alpha3 = input.phase(cont).control(:,3);
beta3 = input.phase(cont).control(:,4);
tp3 = input.phase(cont).control(:,7);

derr3 = input.phase(cont).state(:,1);
err3 = dtheta3;

[dalpha3,dbeta3,PHI,dPHI] = idof_transform(alpha3,beta3,theta3,dtheta3,input.auxdata.PAR);
q3 = [alpha3,beta3,theta3];
qd3 = [dalpha3,dbeta3,dtheta3];

% [M3,ke3,~,H3,k3] = four_bar_system(q3, qd3,input.auxdata.PAR);
[M3,ke3,~,H3,k3] = four_bar_system_no_motor(q3, qd3,input.auxdata.PAR);
% fator = fator + integrate(input.phase(cont).integrand,100*exp(-((q3(:,1)-q3(:,2))/0.2).^2));
K = PHI'*M3*dPHI*dtheta3;
u3 = (Ci*input.auxdata.PAR.Rr^2*err3+(Mi*input.auxdata.PAR.Rr^2)*(1/Td)*(err3-derr3)+Fri*input.auxdata.PAR.Rr);
output.phase(cont).RightHandSide = PHI'*(H3*[tau_s3;tau_e3]+ke3-k3)-K+u3;
output.phase(cont).MassMatrix = PHI'*M3*PHI;

ddtheta3 = output.phase(cont).RightHandSide./output.phase(cont).MassMatrix;
qdd3 = dPHI*dtheta3 + PHI*ddtheta3;
Tp3 = M3(3,:)*qdd3-(ke3(3)-k3(3)+u3);

output.phase(cont).path = [q3(:,1)-q3(:,2), ...
    input.auxdata.PAR.A*sin(alpha3)+input.auxdata.PAR.B*sin(beta3)-input.auxdata.PAR.R*sin(theta3)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha3)+input.auxdata.PAR.B*cos(beta3)-input.auxdata.PAR.R*cos(theta3)-input.auxdata.PAR.h, Tp3 - tp3];
output.phase(cont).derivatives = (1/Td)*(err3-derr3);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice)./(input.phase(cont).final.time-input.phase(cont).initial.time);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice)./(input.phase(cont).final.time-input.phase(cont).initial.time);
cont = cont + 1;

q4 = input.phase(cont).position;
qd4 = input.phase(cont).velocity;
tau_s4 = input.phase(cont).control(:,1)-input.phase(cont).control(:,3);
tau_e4 = input.phase(cont).control(:,2)-input.phase(cont).control(:,4);

derr4 = input.phase(cont).state(:,1);
err4 = qd4(3);
u4 = (Ci*input.auxdata.PAR.Rr^2*err4+(Mi*input.auxdata.PAR.Rr^2)*(1/Td)*(err4-derr4)+Fri*input.auxdata.PAR.Rr);

% [M4,ke4,~,H4,k4] = four_bar_system(q4, qd4,input.auxdata.PAR);
[M4,ke4,~,H4,k4] = four_bar_system_no_motor(q4, qd4,input.auxdata.PAR);
% fator = fator + integrate(input.phase(cont).integrand,100*exp(-((q4(:,1)-q4(:,2))/0.2).^2));
output.phase(cont).RightHandSide = H4*[tau_s4;tau_e4] + ke4 -k4 + [0;0;u4];
output.phase(cont).MassMatrix = M4;
output.phase(cont).path = q4(:,1)-q4(:,2);
output.phase(cont).derivatives = (1/Td)*(err4-derr4);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,3).^indice+input.phase(cont).control(:,4).^indice);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice)./(input.phase(cont).final.time-input.phase(cont).initial.time);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,3).^indice+input.phase(cont).control(:,4).^indice)./(input.phase(cont).final.time-input.phase(cont).initial.time);

[~,~,PHI2i,~] = idof_transform(input.phase(cont).initial.position(1),input.phase(cont).initial.position(2),input.phase(cont).initial.position(3),input.phase(cont).initial.velocity(3),input.auxdata.PAR);
[~,~,PHI2f,~] = idof_transform(input.phase(cont).final.position(1),input.phase(cont).final.position(2),input.phase(cont-1).initial.position,input.phase(cont).final.velocity(3),input.auxdata.PAR);
p_constraints = input.phase(cont).initial.time-input.phase(cont-1).final.time;
p_constraints = [p_constraints,input.phase(cont).initial.position(3)-input.phase(cont-1).final.position, ...
    input.auxdata.PAR.A*sin(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(cont).initial.position(3))-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(cont).initial.position(3))-input.auxdata.PAR.h, ...
    input.phase(cont).initial.velocity-PHI2i'*input.phase(cont-1).final.velocity, ...
    input.auxdata.PAR.A*sin(input.phase(cont).final.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).final.position(2))-input.auxdata.PAR.R*sin(input.phase(cont-1).initial.position)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).final.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).final.position(2))-input.auxdata.PAR.R*cos(input.phase(cont-1).initial.position)-input.auxdata.PAR.h, ...
    input.phase(cont).final.velocity-PHI2f'*input.phase(cont-1).initial.velocity, ...
    input.phase(cont-1).final.state-input.phase(cont).initial.state, ...
    input.phase(cont).final.state-input.phase(cont-1).initial.state, ...
    input.phase(cont-1).final.time-input.phase(cont-1).initial.time, ...
    input.phase(cont).final.time-input.phase(cont).initial.time, ...
    (input.phase(cont).final.position(3)-input.phase(cont-1).initial.position)*input.auxdata.PAR.Rr-input.auxdata.speed*(input.phase(cont).final.time-input.phase(cont-1).initial.time), ...
	input.phase(1).final.position-input.phase(1).initial.position];

%      ...
% theta_total =  input.phase(cont).final.position(3)-input.phase(cont-1).initial.position(1);
output.constraints = p_constraints;
% obj = obj+1e3*(sl1.^2+sl2.^2);
% output.constraints = [input.phase(2).initial.time-input.phase(1).final.time,input.phase(2).final.time-input.phase(2).initial.time, ...
%     input.phase(1).final.position(3)-input.phase(1).final.position(2),input.phase(2).final.position(3)-input.phase(2).final.position(2)];
output.objective = obj./(input.phase(cont).final.time-input.phase(1).initial.time);