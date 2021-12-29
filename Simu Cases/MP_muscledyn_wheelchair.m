function output = MP_muscledyn_wheelchair(PAR,ncycle,velocity,distance,varargin)


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
medtime = distance/(velocity*(ncycle*2+1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Act_v = 0;
max_velang = 500;
t_ini = -120*pi/180; % -120*pi/180 - PAR.thetac1
mintime = 0.1;
maxalpha = 160*pi/180;
lbnd = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

setup.auxdata.PAR = PAR;
setup.auxdata.ncycle = ncycle;
% setup.auxdata.alphai  = alphai;
% setup.auxdata.betai  = betai;
% setup.auxdata.iJi = [-(PAR.R*sin(betai - PAR.thetai))/(PAR.A*sin(alphai - betai)),(PAR.R*sin(alphai - PAR.thetai))/(PAR.B*sin(alphai - betai)),1];
p = inputParser;


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
addParameter(p,'FixedEndTt',false,@islogical);

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
hasFixedEndTt = p.Results.FixedEndTt;
if MShoulder < 0
	MShoulder = MContr;
end
MElbow = p.Results.Max_elbow;
if MElbow < 0
	MElbow = MContr;
end
if hasFixedStart
	hFS = 0;
else
	hFS = inf;
end
if hasFixedEndTt
	hFT = 0;
else
	hFT = inf;
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
    setup.bound.lower.phase(cnt).initial.state = [0,0,0,0];
    setup.bound.upper.phase(cnt).initial.state = [1,1,1,1];
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
    setup.bound.lower.phase(cnt).final.state = [0,0,0,0];
    setup.bound.upper.phase(cnt).final.state = [1,1,1,1];
	setup.bound.lower.phase(cnt).position = PAR.thetac1;
	setup.bound.upper.phase(cnt).position = PAR.thetac2;
	setup.bound.lower.phase(cnt).velocity = -50;
	setup.bound.upper.phase(cnt).velocity = +50;
    setup.bound.lower.phase(cnt).state = [0,0,0,0];
    setup.bound.upper.phase(cnt).state = [1,1,1,1];
	setup.bound.lower.phase(cnt).control = [lbnd, lbnd,  30*pi/180,-pi/2, lbnd, lbnd,-inf,        0,     0,        0,     0];
	setup.bound.upper.phase(cnt).control = [   1,    1,   maxalpha,   pi,    1,    1,+inf,MShoulder,MElbow,MShoulder,MElbow];
	setup.bound.lower.phase(cnt).path = [  0*pi/180,0,0,0,-Act_v,-Act_v,-Act_v,-Act_v];
	setup.bound.upper.phase(cnt).path = [120*pi/180,0,0,0,+Act_v,+Act_v,+Act_v,+Act_v];
	setup.phase(cnt).mesh_points = 10;
	setup.phase(cnt).mesh_number = 10;
end

% Recovery
for i = 1:ncycle
	cnt = 2*i;
	setup.bound.lower.phase(cnt).initial.position = [ 30*pi/180,-pi/2,PAR.thetac1];
	setup.bound.upper.phase(cnt).initial.position = [  maxalpha,   pi,PAR.thetac2];
	setup.bound.lower.phase(cnt).initial.velocity = [-max_velang,-max_velang, 0];
	setup.bound.upper.phase(cnt).initial.velocity = [+max_velang,+max_velang,50];
	setup.bound.lower.phase(cnt).initial.time = 0.1;
	setup.bound.upper.phase(cnt).initial.time = inf;
    setup.bound.lower.phase(cnt).initial.state = [0,0,0,0];
    setup.bound.upper.phase(cnt).initial.state = [1,1,1,1];
	% setup.bound.lower.phase(cnt).final.position = [0,-pi/2,-150*pi/180];
	setup.bound.lower.phase(cnt).final.position = [ 30*pi/180,-pi/2,PAR.thetac1];
	setup.bound.upper.phase(cnt).final.position = [  maxalpha,   pi,        inf];
	setup.bound.lower.phase(cnt).final.velocity = [-max_velang,-max_velang, 0];
	setup.bound.upper.phase(cnt).final.velocity = [+max_velang,+max_velang,50];
	setup.bound.lower.phase(cnt).final.time = 0.2;
	setup.bound.upper.phase(cnt).final.time = inf;
    setup.bound.lower.phase(cnt).final.state = [0,0,0,0];
    setup.bound.upper.phase(cnt).final.state = [1,1,1,1];
	setup.bound.lower.phase(cnt).position = [ 30*pi/180,-pi/2,PAR.thetac1];
	setup.bound.upper.phase(cnt).position = [  maxalpha,   pi,        inf];
	setup.bound.lower.phase(cnt).velocity = [-max_velang,-max_velang, 0];
	setup.bound.upper.phase(cnt).velocity = [+max_velang,+max_velang,50];
    setup.bound.lower.phase(cnt).state = [0,0,0,0];
    setup.bound.upper.phase(cnt).state = [1,1,1,1];
	setup.bound.lower.phase(cnt).control = [lbnd,lbnd,lbnd,lbnd,0,0,0,0];
	setup.bound.upper.phase(cnt).control = [1,1,1,1,MShoulder,MElbow,MShoulder,MElbow];
	setup.bound.lower.phase(cnt).path = [  0*pi/180,-Act_v,-Act_v,-Act_v,-Act_v];
	setup.bound.upper.phase(cnt).path = [120*pi/180,+Act_v,+Act_v,+Act_v,+Act_v];
	setup.phase(cnt).mesh_points = 10;
	setup.phase(cnt).mesh_number = 10;
end

% Parameter
setup.bound.lower.parameter = [PAR.thetac1];
setup.bound.upper.parameter = [PAR.thetac2];

% Start Conditions
setup.bound.lower.phase(1).initial.velocity = 0;
setup.bound.upper.phase(1).initial.velocity = 0;
setup.bound.lower.phase(1).initial.time = 0;
setup.bound.upper.phase(1).initial.time = 0;
setup.bound.lower.phase(1).initial.state = [0,0,0,0];
setup.bound.upper.phase(1).initial.state = [0,0,0,0];
setup.phase(1).mesh_points = 10;
setup.phase(1).mesh_number = 10;

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
	ub = [ub,zeros(1,21),    inf,    inf,    inf,+hFS,+hFT];
	lb = [lb,zeros(1,21),mintime,mintime,Contact,-hFS,-hFT];
end
setup.bound.lower.pconstraints = [lb,mintime,velocity,-hFS];
setup.bound.upper.pconstraints = [ub,    inf,velocity,+hFS];

%% Initial Guess
% Propulsion
for i = 0:ncycle
	cnt = 2*i+1;
    numl = numel(opt.solution.phase(cnt).time);
	setup.initial_guess.phase(cnt).time = opt.solution.phase(cnt).time;
	setup.initial_guess.phase(cnt).position = opt.solution.phase(cnt).position;
	setup.initial_guess.phase(cnt).velocity = opt.solution.phase(cnt).velocity;
	if size(opt.solution.phase(cnt).control,2) == 11
		setup.initial_guess.phase(cnt).control = [opt.solution.phase(cnt).control];
	else
		setup.initial_guess.phase(cnt).control(:,1) = opt.solution.phase(cnt).control(:,1)/max(opt.solution.phase(cnt).control(:,1)+0.01);
		setup.initial_guess.phase(cnt).control(:,2) = opt.solution.phase(cnt).control(:,2)/max(opt.solution.phase(cnt).control(:,2)+0.01);
		setup.initial_guess.phase(cnt).control(:,3:4) = opt.solution.phase(cnt).control(:,3:4);
		setup.initial_guess.phase(cnt).control(:,5) = opt.solution.phase(cnt).control(:,5)/max(opt.solution.phase(cnt).control(:,5)+0.01);
		setup.initial_guess.phase(cnt).control(:,6) = opt.solution.phase(cnt).control(:,6)/max(opt.solution.phase(cnt).control(:,6)+0.01);
		setup.initial_guess.phase(cnt).control(:,7) = opt.solution.phase(cnt).taup;
		setup.initial_guess.phase(cnt).control(:,8:9) = opt.solution.phase(cnt).control(:,1:2);
		setup.initial_guess.phase(cnt).control(:,10:11) = opt.solution.phase(cnt).control(:,5:6);   
    end
    if isfield(opt.solution.phase(cnt),'state')
        setup.initial_guess.phase(cnt).state = [opt.solution.phase(cnt).state];
    else
        setup.initial_guess.phase(cnt).state(:,1) = opt.solution.phase(cnt).control(:,1)/max(opt.solution.phase(cnt).control(:,1)+0.01);
		setup.initial_guess.phase(cnt).state(:,2) = opt.solution.phase(cnt).control(:,2)/max(opt.solution.phase(cnt).control(:,2)+0.01);
        setup.initial_guess.phase(cnt).state(:,3) = opt.solution.phase(cnt).control(:,5)/max(opt.solution.phase(cnt).control(:,5)+0.01);
		setup.initial_guess.phase(cnt).state(:,4) = opt.solution.phase(cnt).control(:,6)/max(opt.solution.phase(cnt).control(:,6)+0.01);
    end
end
	
% Recovery
for i = 1:ncycle
	cnt = 2*i;
    numl = numel(opt.solution.phase(cnt).time);
	setup.initial_guess.phase(cnt).time = opt.solution.phase(cnt).time;
	setup.initial_guess.phase(cnt).position = opt.solution.phase(cnt).position;
	setup.initial_guess.phase(cnt).velocity = opt.solution.phase(cnt).velocity;
	if size(opt.solution.phase(cnt).control,2) == 8
		setup.initial_guess.phase(cnt).control = [opt.solution.phase(cnt).control];
	else
		setup.initial_guess.phase(cnt).control(:,1) = opt.solution.phase(cnt).control(:,1)/max(opt.solution.phase(cnt).control(:,1)+0.01);
		setup.initial_guess.phase(cnt).control(:,2) = opt.solution.phase(cnt).control(:,2)/max(opt.solution.phase(cnt).control(:,2)+0.01);
		setup.initial_guess.phase(cnt).control(:,3) = opt.solution.phase(cnt).control(:,3)/max(opt.solution.phase(cnt).control(:,3)+0.01);
		setup.initial_guess.phase(cnt).control(:,4) = opt.solution.phase(cnt).control(:,4)/max(opt.solution.phase(cnt).control(:,4)+0.01);
		setup.initial_guess.phase(cnt).control(:,5:8) = opt.solution.phase(cnt).control(:,1:4);   
    end
    if isfield(opt.solution.phase(cnt),'state')
        setup.initial_guess.phase(cnt).state = [opt.solution.phase(cnt).state];
    else
        setup.initial_guess.phase(cnt).state(:,1) = opt.solution.phase(cnt).control(:,1)/max(opt.solution.phase(cnt).control(:,1)+0.01);
		setup.initial_guess.phase(cnt).state(:,2) = opt.solution.phase(cnt).control(:,2)/max(opt.solution.phase(cnt).control(:,2)+0.01);
        setup.initial_guess.phase(cnt).state(:,3) = opt.solution.phase(cnt).control(:,3)/max(opt.solution.phase(cnt).control(:,3)+0.01);
		setup.initial_guess.phase(cnt).state(:,4) = opt.solution.phase(cnt).control(:,4)/max(opt.solution.phase(cnt).control(:,4)+0.01);
    end
end

% Parameter
setup.initial_guess.parameter = -100*pi/180;

%% Run Program
output = ddiopt_MB(setup);

%% Options used in this simulation
output.Options.Type = 'MSD-Reference';
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
    % q1 = vect([output.solution.phase(cnt).control(:,3:4),output.solution.phase(cnt).position]);
    % [da,db,PHI,dPHI] = idof_transform(vect(output.solution.phase(cnt).control(:,3)),vect(output.solution.phase(cnt).control(:,4)),vect(output.solution.phase(cnt).position),vect(output.solution.phase(cnt).velocity),PAR);
    % qd1 = [da,db,vect(output.solution.phase(cnt).velocity)];
    % [M1,ke1,~,H1,k1] = four_bar_system_no_motor(q1, qd1, PAR);
    
    % K = PHI'*(ke1+H1*[vect(output.solution.phase(cnt).control(:,1))-vect(output.solution.phase(cnt).control(:,5));vect(output.solution.phase(cnt).control(:,2))-vect(output.solution.phase(cnt).control(:,6))]-k1);
    % MM = PHI'*M1*PHI;
    % ddt = (K-PHI'*M1*dPHI*vect(output.solution.phase(cnt).velocity))./MM;
    % qdd = PHI*ddt+dPHI*vect(output.solution.phase(cnt).velocity);
    % tp = M1(3,:)*qdd-(ke1(3)+H1(3,:)*[vect(output.solution.phase(cnt).control(:,1))-vect(output.solution.phase(cnt).control(:,5));vect(output.solution.phase(cnt).control(:,2))-vect(output.solution.phase(cnt).control(:,6))]-k1(3));
    % output.solution.phase(cnt).taup = double(tp);
	output.solution.phase(cnt).taup = output.solution.phase(cnt).control(:,7);
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
tact = 0.02; % 0.01
tdat = 0.04; % 0.04

%%% Original coordinates system: Extension is positive, Flexion is negative
Qv  = cell(1,2*ncycle+1);
Qdv = cell(1,2*ncycle+1);
act = cell(1,2*ncycle+1);
tau = cell(1,2*ncycle+1);
dac = cell(1,2*ncycle+1);
PHI = cell(1,2*ncycle+1);
dPHI= cell(1,2*ncycle+1);
TA  = cell(1,2*ncycle+1);
TP  = cell(1,2*ncycle+1);
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
	
	ds_e  = input.phase(cnt).control(:,1);
	ds_f  = input.phase(cnt).control(:,5);
	de_e  = input.phase(cnt).control(:,2);
	de_f  = input.phase(cnt).control(:,6);
	dac{cnt} = [ds_e;ds_f;de_e;de_f];
    as_e  = input.phase(cnt).state(:,1);
	as_f  = input.phase(cnt).state(:,3);
	ae_e  = input.phase(cnt).state(:,2);
	ae_f  = input.phase(cnt).state(:,4);
	act{cnt} = [as_e;as_f;ae_e;ae_f];
	ts_e  = input.phase(cnt).control(:,8);
	ts_f  = input.phase(cnt).control(:,10);
	te_e  = input.phase(cnt).control(:,9);
	te_f  = input.phase(cnt).control(:,11);
	tau{cnt} = [ts_e;ts_f;te_e;te_f];
end

%% Recovery
for i=1:input.auxdata.ncycle
	cnt = 2*i;
	
	Qv{cnt}  = input.phase(cnt).position;
	Qdv{cnt} = input.phase(cnt).velocity;
	
    ds_e  = input.phase(cnt).control(:,1);
	ds_f  = input.phase(cnt).control(:,3);
	de_e  = input.phase(cnt).control(:,2);
	de_f  = input.phase(cnt).control(:,4);
	dac{cnt} = [ds_e;ds_f;de_e;de_f];
	as_e  = input.phase(cnt).state(:,1);
	as_f  = input.phase(cnt).state(:,3);
	ae_e  = input.phase(cnt).state(:,2);
	ae_f  = input.phase(cnt).state(:,4);
	act{cnt} = [as_e;as_f;ae_e;ae_f];
	ts_e  = input.phase(cnt).control(:,5);
	ts_f  = input.phase(cnt).control(:,7);
	te_e  = input.phase(cnt).control(:,6);
	te_f  = input.phase(cnt).control(:,8);
	tau{cnt} = [ts_e;ts_f;te_e;te_f];
end

%% Activation to Torque Constraints
for i=1:2*input.auxdata.ncycle+1
	[ap1, bt1] = dof_conversion(Qv{i}(1),Qv{i}(2),'Type', 'angle');
	[dap1, dbt1] = dof_conversion(Qdv{i}(1),Qdv{i}(2),'Type', 'angvel');
	[~,~,TA{i},TP{i}] = MTG_smooth(act{i}(2),act{i}(1),act{i}(4),act{i}(3),ap1,bt1,dap1,dbt1);
    TA{i} = 2*TA{i}; TP{i} = 2*TP{i}; % two arms depicted as one
end

%% Propulsion
for i=0:input.auxdata.ncycle
	cnt = 2*i+1;
	
	tau_s1 = tau{cnt}(1)-tau{cnt}(2)-2*TP{cnt}(1);
	tau_e1 = tau{cnt}(3)-tau{cnt}(4)-2*TP{cnt}(2);

	[M1,ke1,~,H1,k1] = four_bar_system(Qv{cnt}, Qdv{cnt},input.auxdata.PAR,'Type','Reference');
	MM1 = PHI{cnt}'*M1*PHI{cnt};
	K1  = PHI{cnt}'*M1*dPHI{cnt}*Qdv{cnt}(3);
	KE1 = PHI{cnt}'*(H1*[tau_s1;tau_e1]+ke1-k1);
	qdd1 = MM1\(KE1-K1);
	qdd1 = dPHI{cnt}*Qdv{cnt}(3)+PHI{cnt}*qdd1;
	tp = M1(3,:)*qdd1-ke1(3)+k1(3);
%     tp = H1(3,:)*[tau_s1;tau_e1];
	
	output.phase(cnt).RightHandSide = KE1-K1;
	output.phase(cnt).MassMatrix = MM1;
    output.phase(cnt).derivatives = [(dac{cnt}(1)-act{cnt}(1))*(dac{cnt}(1)/tact+(1-dac{cnt}(1))/tdat), ...
                                     (dac{cnt}(3)-act{cnt}(3))*(dac{cnt}(3)/tact+(1-dac{cnt}(3))/tdat), ...
                                     (dac{cnt}(2)-act{cnt}(2))*(dac{cnt}(2)/tact+(1-dac{cnt}(2))/tdat), ...
                                     (dac{cnt}(4)-act{cnt}(4))*(dac{cnt}(4)/tact+(1-dac{cnt}(4))/tdat)];
	output.phase(cnt).path = [Qv{cnt}(1)-Qv{cnt}(2), ...
		input.auxdata.PAR.A*sin(Qv{cnt}(1))+input.auxdata.PAR.B*sin(Qv{cnt}(2))-input.auxdata.PAR.R*sin(Qv{cnt}(3))-input.auxdata.PAR.Y, ...
		input.auxdata.PAR.A*cos(Qv{cnt}(1))+input.auxdata.PAR.B*cos(Qv{cnt}(2))-input.auxdata.PAR.R*cos(Qv{cnt}(3))-input.auxdata.PAR.h, ...
		tp-input.phase(cnt).control(:,7), ...
		tau{cnt}(2)-TA{cnt}(1), tau{cnt}(1)+TA{cnt}(2), ...
        tau{cnt}(4)-TA{cnt}(3), tau{cnt}(3)+TA{cnt}(4)];
end

%% Recovery
for i=1:input.auxdata.ncycle
	cnt = 2*i;
	
	tau_s2 = tau{cnt}(1)-tau{cnt}(2)-2*TP{cnt}(1);
	tau_e2 = tau{cnt}(3)-tau{cnt}(4)-2*TP{cnt}(2);
	
	[M2,ke2,~,H2,k2] = four_bar_system(Qv{cnt}, Qdv{cnt},input.auxdata.PAR,'Type','Reference');
	output.phase(cnt).RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2;
	output.phase(cnt).MassMatrix = M2;
    output.phase(cnt).derivatives = [(dac{cnt}(1)-act{cnt}(1))*(dac{cnt}(1)/tact+(1-dac{cnt}(1))/tdat), ...
                                     (dac{cnt}(3)-act{cnt}(3))*(dac{cnt}(3)/tact+(1-dac{cnt}(3))/tdat), ...
                                     (dac{cnt}(2)-act{cnt}(2))*(dac{cnt}(2)/tact+(1-dac{cnt}(2))/tdat), ...
                                     (dac{cnt}(4)-act{cnt}(4))*(dac{cnt}(4)/tact+(1-dac{cnt}(4))/tdat)];
	output.phase(cnt).path = [Qv{cnt}(1)-Qv{cnt}(2), ...
		tau{cnt}(2)-TA{cnt}(1), tau{cnt}(1)+TA{cnt}(2), ...
        tau{cnt}(4)-TA{cnt}(3), tau{cnt}(3)+TA{cnt}(4)];
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
        input.phase(prop1).final.state-input.phase(ret).initial.state, ...
        input.phase(ret).final.state-input.phase(prop2).initial.state, ...
		input.phase(prop2).initial.time-input.phase(ret).final.time, ...
		input.phase(prop1).final.time-input.phase(prop1).initial.time, ...
		input.phase(ret).final.time-input.phase(ret).initial.time, ...
		input.phase(prop1).final.position-input.phase(prop1).initial.position, ...
		col(input.phase(prop1).parameter,1) - input.phase(prop1).initial.position, ...
        col(input.phase(prop1).control(7),numel(double(input.phase(prop1).control(7))))];
		
	theta_total = theta_total + input.phase(ret).final.position(3)-input.phase(prop1).initial.position;
end

%% Objective
for i=1:2*input.auxdata.ncycle+1
    obji = 0;
% 	obji = integrate(input.phase(i).integrand,tau{i}(1).^indice+tau{i}(2).^indice);
% 	obji = obji + integrate(input.phase(i).integrand,tau{i}(3).^indice+tau{i}(4).^indice);
%     obji = obji + integrate(input.phase(i).integrand,act{i}(1).^indice+act{i}(2).^indice);
% 	obji = obji + integrate(input.phase(i).integrand,act{i}(3).^indice+act{i}(4).^indice);
    obji = obji + integrate(input.phase(i).integrand,dac{i}(1).^indice+dac{i}(2).^indice);
	obji = obji + integrate(input.phase(i).integrand,dac{i}(3).^indice+dac{i}(4).^indice);
	% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);
	% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);
%     obji = obji./(input.phase(i).final.time-input.phase(i).initial.time);
    obj = obj + obji;
end

% for cyc=0:input.auxdata.ncycle
%     obji = 0;
%     if cyc == input.auxdata.ncycle
%         endi = 1;
%     else 
%         endi = 2;
%     end
%     for i=1:endi
% % 	obji = obji + integrate(input.phase(2*cyc+i).integrand,tau{2*cyc+i}(1).^indice+tau{2*cyc+i}(2).^indice);
% % 	obji = obji + integrate(input.phase(2*cyc+i).integrand,tau{2*cyc+i}(3).^indice+tau{2*cyc+i}(4).^indice);
%     obji = obji + integrate(input.phase(2*cyc+i).integrand,act{2*cyc+i}(1).^indice+act{2*cyc+i}(2).^indice);
% 	obji = obji + integrate(input.phase(2*cyc+i).integrand,act{2*cyc+i}(3).^indice+act{2*cyc+i}(4).^indice);
%     end
%     obji = obji./(input.phase(2*cyc+endi).final.time-input.phase(2*cyc+1).initial.time);
%     obj = obj + obji;
% end

% output.objective = obj./(input.phase(end).final.time-input.phase(1).initial.time);
output.objective = obj;

%% Final Link
theta_total = (theta_total+input.phase(prop2).final.position-input.phase(prop2).initial.position);
mean_velocity = theta_total*input.auxdata.PAR.Rr./input.phase(prop2).final.time;
p_constraints = [p_constraints, input.phase(prop2).final.time-input.phase(prop2).initial.time, ...
    mean_velocity, ...
    col(input.phase(prop1).parameter,1) - input.phase(prop2).initial.position];
output.constraints = p_constraints;