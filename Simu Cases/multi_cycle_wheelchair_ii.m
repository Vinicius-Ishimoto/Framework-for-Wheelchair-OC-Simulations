function output =  multi_cycle_wheelchair_ii(PAR,ncycle,velocity,distance,varargin)


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

setup.auxdata.PAR = PAR;
setup.auxdata.ncycle = ncycle;
% setup.auxdata.alphai  = alphai;
% setup.auxdata.betai  = betai;
% setup.auxdata.iJi = [-(PAR.R*sin(betai - PAR.thetai))/(PAR.A*sin(alphai - betai)),(PAR.R*sin(alphai - PAR.thetai))/(PAR.B*sin(alphai - betai)),1];
t_ini = -120*pi/180;
p = inputParser;
medtime = distance/(velocity*(ncycle*2+1));
mintime = 0.1;

addParameter(p,'Guess',-1);
addParameter(p,'Contact_angle',0,@(x) assert(x*pi/180<PAR.thetac2-PAR.thetac1,['Contact angle need to be smaller than ',num2str((PAR.thetac2-PAR.thetac1)*180/pi),' degrees.']));
addParameter(p,'thetaf',PAR.thetaf*180/pi,@(x) assert(x*pi/180<PAR.thetac2,['Contact angle need to be smaller than ',num2str((PAR.thetac2)*180/pi),' degrees.']));
addParameter(p,'Max_control',inf,@isnumeric);
addParameter(p,'Max_shoulder',-1,@isnumeric);
addParameter(p,'Max_elbow',-1,@isnumeric);
addParameter(p,'indice',2,@isnumeric);
addParameter(p,'MinMax',false,@islogical);
addParameter(p,'Fixed',false,@islogical);
addParameter(p,'FixedStart',false,@islogical);

FixedDist = true;
if distance <= 0
    FixedDist = false;
end
parse(p,varargin{:});
opt = p.Results.Guess;
Contact = p.Results.Contact_angle*pi/180;
MContr = p.Results.Max_control;
MShoulder = p.Results.Max_shoulder;
isFixed = p.Results.Fixed;
hasFixedStart = p.Results.FixedStart;
if MShoulder < 0
	MShoulder = MContr;
end
MElbow = p.Results.Max_elbow;
if MElbow < 0
	MElbow = MContr;
end

setup.function = @Dynamics_Multi;
setup.auxdata.indice = p.Results.indice;
PAR.thetaf = p.Results.thetaf;
setup.auxdata.PAR.thetaf = p.Results.thetaf;

%% Boundary Constraints
% Propulsion
for i = 0:ncycle
	cnt = 2*i+1;
		setup.bound.lower.phase(cnt).initial.velocity = 0;
		setup.bound.upper.phase(cnt).initial.velocity = 50;
		setup.bound.lower.phase(cnt).initial.time = 0;
		setup.bound.upper.phase(cnt).initial.time = inf;
	if not(isFixed)
		setup.bound.lower.phase(cnt).initial.position = t_ini;
		setup.bound.upper.phase(cnt).initial.position = PAR.thetac2;
		setup.bound.lower.phase(cnt).final.position = PAR.thetac1;
		setup.bound.upper.phase(cnt).final.position = PAR.thetac2;    
		
	else
		setup.bound.lower.phase(cnt).initial.position = PAR.thetai;
		setup.bound.upper.phase(cnt).initial.position = PAR.thetai;
		setup.bound.lower.phase(cnt).final.position = PAR.thetac1;
		setup.bound.upper.phase(cnt).final.position = PAR.thetac2;
	end
	
	setup.bound.lower.phase(cnt).final.velocity = 0;
	setup.bound.upper.phase(cnt).final.velocity = 50;
	setup.bound.lower.phase(cnt).final.time = 0.1;
	setup.bound.upper.phase(cnt).final.time = inf;
	setup.bound.lower.phase(cnt).position = PAR.thetac1;
	setup.bound.upper.phase(cnt).position = PAR.thetac2;
	setup.bound.lower.phase(cnt).velocity = 0;
	setup.bound.upper.phase(cnt).velocity = 50;
	setup.bound.lower.phase(cnt).control = [        0,     0, 0,-pi/2,        0,     0];
	setup.bound.upper.phase(cnt).control = [MShoulder,MElbow,pi,   pi,MShoulder,MElbow];
	setup.bound.lower.phase(cnt).path = [0,0,0];
	setup.bound.upper.phase(cnt).path = [pi,0,0];
	setup.phase(cnt).mesh_points = 10;
	setup.phase(cnt).mesh_number = 10;
end

% Recovery
for i = 1:ncycle
	cnt = 2*i;
	setup.bound.lower.phase(cnt).initial.position = [0,-pi/2,PAR.thetac1];
	setup.bound.upper.phase(cnt).initial.position = [pi,pi,PAR.thetac2];
	setup.bound.lower.phase(cnt).initial.velocity = [-10*pi,-10*pi,0];
	setup.bound.upper.phase(cnt).initial.velocity = [10*pi,10*pi,50];
	setup.bound.lower.phase(cnt).initial.time = 0.1;
	setup.bound.upper.phase(cnt).initial.time = inf;
	% setup.bound.lower.phase(cnt).final.position = [0,-pi/2,-150*pi/180];
	setup.bound.lower.phase(cnt).final.position = [0,-pi/2,PAR.thetac1];
	setup.bound.upper.phase(cnt).final.position = [pi,pi,inf];
	setup.bound.lower.phase(cnt).final.velocity = [-10*pi,-10*pi,0];
	setup.bound.upper.phase(cnt).final.velocity = [10*pi,10*pi,50];
	setup.bound.lower.phase(cnt).final.time = 0.2;
	setup.bound.upper.phase(cnt).final.time = inf;
	setup.bound.lower.phase(cnt).position = [0,-pi/2,PAR.thetac1];
	setup.bound.upper.phase(cnt).position = [pi,pi,inf];
	setup.bound.lower.phase(cnt).velocity = [-10*pi,-10*pi,0];
	setup.bound.upper.phase(cnt).velocity = [10*pi,10*pi,50];
	setup.bound.lower.phase(cnt).control = [0,0,0,0];
	setup.bound.upper.phase(cnt).control = [MShoulder,MElbow,MShoulder,MElbow];
	setup.bound.lower.phase(cnt).path = 0;
	setup.bound.upper.phase(cnt).path = pi;
	setup.phase(cnt).mesh_points = 12;
	setup.phase(cnt).mesh_number = 12;
end

% Parameter
setup.bound.lower.parameter = [PAR.thetac1];
setup.bound.upper.parameter = [PAR.thetac2];

% Start Conditions
setup.bound.lower.phase(1).initial.velocity = 0;
setup.bound.upper.phase(1).initial.velocity = 0;
setup.bound.lower.phase(1).initial.time = 0;
setup.bound.upper.phase(1).initial.time = 0;

% Final Time
if FixedDist
    setup.bound.lower.phase(end).final.time = distance/velocity;
    setup.bound.upper.phase(end).final.time = distance/velocity;
else
    setup.bound.lower.phase(end).final.time = 0.2;
    setup.bound.upper.phase(end).final.time = inf;
end

% Point Constraints
ub = [];
lb = [];
for i = 1:ncycle
	if hasFixedStart
		ub = [ub,zeros(1,13),    inf,    inf,    inf,0];
		lb = [lb,zeros(1,13),mintime,mintime,Contact,0];
	else
		ub = [ub,zeros(1,13),    inf,    inf,    inf, inf];
		lb = [lb,zeros(1,13),mintime,mintime,Contact,-inf];
	end
end

if hasFixedStart
	setup.bound.lower.pconstraints = [lb,  0.1,velocity,  1,0];
	setup.bound.upper.pconstraints = [ub,  inf,velocity,inf,0];
else
	setup.bound.lower.pconstraints = [lb,  0.1,velocity,  1,-inf];
	setup.bound.upper.pconstraints = [ub,  inf,velocity,inf, inf];
end

%% Initial Guess
% Propulsion
for i = 0:ncycle
	cnt = 2*i+1;
	setup.initial_guess.phase(cnt).time = opt.solution.phase(cnt).time;
	setup.initial_guess.phase(cnt).position = opt.solution.phase(cnt).position;
	setup.initial_guess.phase(cnt).velocity = opt.solution.phase(cnt).velocity;
	if size(opt.solution.phase(cnt).control,2) == 6
		setup.initial_guess.phase(cnt).control = [opt.solution.phase(cnt).control];
	else
		setup.initial_guess.phase(cnt).control = [opt.solution.phase(cnt).control,-opt.solution.phase(cnt).control(:,1:2)];    
    end
end
	
% Recovery
for i = 1:ncycle
	cnt = 2*i;	
	setup.initial_guess.phase(cnt).time = opt.solution.phase(cnt).time;
	setup.initial_guess.phase(cnt).position = opt.solution.phase(cnt).position;
	setup.initial_guess.phase(cnt).velocity = opt.solution.phase(cnt).velocity;
	if size(opt.solution.phase(cnt).control,2) == 4
		setup.initial_guess.phase(cnt).control = [opt.solution.phase(cnt).control];
	else
		setup.initial_guess.phase(cnt).control = [opt.solution.phase(cnt).control,-opt.solution.phase(cnt).control(:,1:2)];    
	end
end

% Parameter
setup.initial_guess.parameter = -100*pi/180;

%% Run Program
output = ddiopt_MB(setup);

%% Options used in this simulation
output.Options.Type = 'Reference';
output.Options.NumCycles = ncycle;
output.Options.TotalDistance = distance;
output.Options.MeanVelocity = velocity;
output.Options.Guess = setup.initial_guess;
output.Options.MinContactAngle = Contact;
output.Options.thetaf = PAR.thetaf;
output.Options.MaxControl = MContr;
output.Options.MaxShoulder = MShoulder;
output.Options.MaxElbow = MElbow;
output.Options.Indice = setup.auxdata.indice;
output.Options.FixedContactAngle = isFixed;
output.Options.FixedStartAngle = hasFixedStart;

%% Return tau_p
% Propulsion
for i = 0:ncycle
	cnt = 2*i+1;
    q1 = vect([output.solution.phase(cnt).control(:,3:4),output.solution.phase(cnt).position]);
    [da,db,PHI,dPHI] = idof_transform(vect(output.solution.phase(cnt).control(:,3)),vect(output.solution.phase(cnt).control(:,4)),vect(output.solution.phase(cnt).position),vect(output.solution.phase(cnt).velocity),PAR);
    qd1 = [da,db,vect(output.solution.phase(cnt).velocity)];
    [M1,ke1,~,H1,k1] = four_bar_system_no_motor(q1, qd1, PAR);
    
    K = PHI'*(ke1+H1*[vect(output.solution.phase(cnt).control(:,1))-vect(output.solution.phase(cnt).control(:,5));vect(output.solution.phase(cnt).control(:,2))-vect(output.solution.phase(cnt).control(:,6))]-k1);
    MM = PHI'*M1*PHI;
    ddt = (K-PHI'*M1*dPHI*vect(output.solution.phase(cnt).velocity))./MM;
    qdd = PHI*ddt+dPHI*vect(output.solution.phase(cnt).velocity);
    tp = M1(3,:)*qdd-(ke1(3)+H1(3,:)*[vect(output.solution.phase(cnt).control(:,1))-vect(output.solution.phase(cnt).control(:,5));vect(output.solution.phase(cnt).control(:,2))-vect(output.solution.phase(cnt).control(:,6))]-k1(3));
    output.solution.phase(cnt).taup = double(tp);
end
% Recovery
for i = 1:ncycle
	cnt = 2*i;
    output.solution.phase(cnt).taup = zeros(numel(output.solution.phase(cnt).velocity(:,1)),1);
end

%% Internal Function
function output = Dynamics_Multi(input)

p_constraints = [];
obj = 0;
theta_total = 0;
cont = 1;
ncycle = input.auxdata.ncycle;
indice = input.auxdata.indice;

%%% Original coordinates system: Extension is positive, Flexion is negative
Qv  = cell(1,2*ncycle+1);
Qdv = cell(1,2*ncycle+1);
tau = cell(1,2*ncycle+1);
PHI = cell(1,2*ncycle+1);
dPHI= cell(1,2*ncycle+1);
%% Propulsion
for i=0:input.auxdata.ncycle
	cnt = 2*i+1;
	
	theta1 = input.phase(cnt).position;
	dtheta1 = input.phase(cnt).velocity;
	alpha1 = input.phase(cnt).control(:,3);
	beta1 = input.phase(cnt).control(:,4);
	[dalpha1,dbeta1,PHI{cnt},dPHI{cnt}] = idof_transform(alpha1,beta1,theta1,dtheta1,input.auxdata.PAR);
	Qv{cnt}  = [alpha1,beta1,theta1];
	Qdv{cnt} = [dalpha1,dbeta1,dtheta1];
	
	ts_e  = input.phase(cnt).control(:,1);
	ts_f  = input.phase(cnt).control(:,5);
	te_e  = input.phase(cnt).control(:,2);
	te_f  = input.phase(cnt).control(:,6);
	tau{cnt} = [ts_e;ts_f;te_e;te_f];
end

%% Recovery
for i=1:input.auxdata.ncycle
	cnt = 2*i;
	
	Qv{cnt}  = input.phase(cnt).position;
	Qdv{cnt} = input.phase(cnt).velocity;
	
	ts_e  = input.phase(cnt).control(:,1);
	ts_f  = input.phase(cnt).control(:,3);
	te_e  = input.phase(cnt).control(:,2);
	te_f  = input.phase(cnt).control(:,4);
	tau{cnt} = [ts_e;ts_f;te_e;te_f];
end

%% Propulsion
for i=0:input.auxdata.ncycle
	cnt = 2*i+1;
	tau_s1 = tau{cnt}(1)-tau{cnt}(2);
	tau_e1 = tau{cnt}(3)-tau{cnt}(4);

	[M1,ke1,~,H1,k1] = four_bar_system_no_motor(Qv{cnt}, Qdv{cnt},input.auxdata.PAR);
	K = PHI{cnt}'*M1*dPHI{cnt}*Qdv{cnt}(3);
	output.phase(cnt).RightHandSide = PHI{cnt}'*(H1*[tau_s1;tau_e1]+ke1-k1)-K;
	output.phase(cnt).MassMatrix = PHI{cnt}'*M1*PHI{cnt};
	output.phase(cnt).path = [Qv{cnt}(1)-Qv{cnt}(2), ...
		input.auxdata.PAR.A*sin(Qv{cnt}(1))+input.auxdata.PAR.B*sin(Qv{cnt}(2))-input.auxdata.PAR.R*sin(Qv{cnt}(3))-input.auxdata.PAR.Y, ...
		input.auxdata.PAR.A*cos(Qv{cnt}(1))+input.auxdata.PAR.B*cos(Qv{cnt}(2))-input.auxdata.PAR.R*cos(Qv{cnt}(3))-input.auxdata.PAR.h];
end

%% Recovery
for i=1:input.auxdata.ncycle
	cnt = 2*i;
	tau_s2 = tau{cnt}(1)-tau{cnt}(2);
	tau_e2 = tau{cnt}(3)-tau{cnt}(4);
	
	[M2,ke2,~,H2,k2] = four_bar_system_no_motor(Qv{cnt}, Qdv{cnt},input.auxdata.PAR);
	output.phase(cnt).RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2;
	output.phase(cnt).MassMatrix = M2;
	output.phase(cnt).path = Qv{cnt}(1)-Qv{cnt}(2);
end

%% Link Between Cycles
for i=1:input.auxdata.ncycle	
	prop1 = 2*i-1;
	ret   = 2*i;
	prop2 = 2*i+1;
	
	[~,~,PHI2i] = idof_transform(input.phase(ret).initial.position(1),input.phase(ret).initial.position(2),input.phase(ret).initial.position(3),input.phase(ret).initial.velocity(3),input.auxdata.PAR);
	[~,~,PHI2f] = idof_transform(input.phase(ret).final.position(1),input.phase(ret).final.position(2),input.phase(prop2).initial.position,input.phase(ret).final.velocity(3),input.auxdata.PAR);
	
	p_constraints = [p_constraints, input.phase(ret).initial.time-input.phase(prop1).final.time];
	p_constraints = [p_constraints,input.phase(ret).initial.position(3)-input.phase(prop1).final.position, ...
		input.auxdata.PAR.A*sin(input.phase(ret).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(ret).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(ret).initial.position(3))-input.auxdata.PAR.Y, ...
		input.auxdata.PAR.A*cos(input.phase(ret).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(ret).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(ret).initial.position(3))-input.auxdata.PAR.h, ...
		input.phase(prop1).final.velocity*PHI2i'-input.phase(ret).initial.velocity, ...
		input.auxdata.PAR.A*sin(input.phase(ret).final.position(1))+input.auxdata.PAR.B*sin(input.phase(ret).final.position(2))-input.auxdata.PAR.R*sin(input.phase(prop2).initial.position)-input.auxdata.PAR.Y, ...
		input.auxdata.PAR.A*cos(input.phase(ret).final.position(1))+input.auxdata.PAR.B*cos(input.phase(ret).final.position(2))-input.auxdata.PAR.R*cos(input.phase(prop2).initial.position)-input.auxdata.PAR.h, ...
		input.phase(ret).final.velocity-PHI2f'*input.phase(prop2).initial.velocity, ...
		input.phase(prop2).initial.time-input.phase(ret).final.time, ...
		input.phase(prop1).final.time-input.phase(prop1).initial.time, ...
		input.phase(ret).final.time-input.phase(ret).initial.time, ...
		input.phase(prop1).final.position-input.phase(prop1).initial.position, ...
		col(input.phase(prop1).parameter,1) - input.phase(prop1).initial.position];
		
	theta_total = theta_total + input.phase(ret).final.position(3)-input.phase(prop1).initial.position;
end

%% Objective
for i=1:2*input.auxdata.ncycle+1
	obj = obj + integrate(input.phase(i).integrand,tau{i}(1).^indice+tau{i}(2).^indice);
	obj = obj + integrate(input.phase(i).integrand,tau{i}(3).^indice+tau{i}(4).^indice);
	% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);
	% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);
end
output.objective = obj./(input.phase(end).final.time-input.phase(1).initial.time);
% output.objective = obj;

%% Final Link
theta_total = (theta_total+input.phase(prop2).final.position-input.phase(prop2).initial.position);
mean_velocity = theta_total*input.auxdata.PAR.Rr./input.phase(prop2).final.time;
p_constraints = [p_constraints, input.phase(prop2).final.time-input.phase(prop2).initial.time, ...
    mean_velocity, ...
    obj, col(input.phase(prop1).parameter,1) - input.phase(prop2).initial.position];
output.constraints = p_constraints;