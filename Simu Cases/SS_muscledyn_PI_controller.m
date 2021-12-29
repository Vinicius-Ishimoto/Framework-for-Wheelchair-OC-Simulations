function output = SS_muscledyn_PI_controller(PAR,speed,varargin)

% PAR = parameters;
setup.auxdata.PAR = PAR;
setup.mesh_points = 5;
setup.mesh_number = 20;
setup.function = @Dynamics_guess;
thetai = PAR.thetai;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Act_v = 0;
max_velang = 500;
t_ini = PAR.thetac1; % -120*pi/180
mintime = 0.1;
maxalpha = 160*pi/180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

p = inputParser;

addParameter(p,'Guess',-1);
addParameter(p,'Contact_angle',0,@(x) assert(x*pi/180<PAR.thetac2-PAR.thetac1,['Contact angle need to be smaller than ',num2str((PAR.thetac2-PAR.thetac1)*180/pi),' degrees.']));
addParameter(p,'thetaf',PAR.thetaf*180/pi,@(x) assert(x*pi/180<PAR.thetac2,['Contact angle need to be smaller than ',num2str((PAR.thetac2)*180/pi),' degrees.']));
addParameter(p,'Max_control',inf,@isnumeric);
addParameter(p,'Max_shoulder',-1,@isnumeric);
addParameter(p,'Max_elbow',-1,@isnumeric);
addParameter(p,'ControllerK',PAR.const,@isnumeric);
addParameter(p,'Mi',PAR.Mi,@isnumeric);
addParameter(p,'Ci',PAR.Ci,@isnumeric);
addParameter(p,'indice',2,@isnumeric);
addParameter(p,'MinMax',false,@islogical);
addParameter(p,'Fixed',false,@islogical);
addParameter(p,'FixedStart',false,@islogical);
addParameter(p,'FixedEndTt',false,@islogical);
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
setup.auxdata.const = p.Results.ControllerK;
setup.auxdata.PAR.Mi = p.Results.Mi;
setup.auxdata.PAR.Ci = p.Results.Ci;

if p.Results.MinMax
    setup.function = @Dynamics_minmax;
end

if hasFixedEndTt
	hFT = 0;
else
	hFT = inf;
end

thetaf = p.Results.thetaf*pi/180;

% speed = 0.5;
setup.auxdata.speed = speed;
setup.auxdata.indice = p.Results.indice;
% k1 = double(-2*PAR.B*(PAR.Y + PAR.R*sin(thetai)));
% k2 = double(2*PAR.B*(-PAR.h - PAR.R*cos(thetai)));
% k3 = double(- PAR.A^2 + PAR.B^2 + PAR.R^2 + 2*sin(thetai)*PAR.R*PAR.Y + 2*cos(thetai)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2);
% betai = 2*atan2(-k1-sqrt(k1^2+k2^2-k3^2),k3-k2);

% k1a = -2*PAR.A*(PAR.Y + PAR.R*sin(thetai));
% k2a = 2*PAR.A*(-PAR.h - PAR.R*cos(thetai));
% k3a = PAR.A^2 - PAR.B^2 + PAR.R^2 + 2*sin(thetai)*PAR.R*PAR.Y + 2*cos(thetai)*PAR.R*PAR.h + PAR.Y^2 + PAR.h^2;
% alphai = 2*atan2(-k1a+sqrt(k1a^2+k2a^2-k3a^2),k3a-k2a);
% inip = fsolve(@(x) [PAR.A*sin(x(1))+PAR.B*sin(x(2))-PAR.R*sin(thetai)-PAR.Y;PAR.A*cos(x(1))+PAR.B*cos(x(2))-PAR.R*cos(PAR.thetai)-PAR.h],[2.1726;0.5903]);
% inip = [alphai;betai];
% setup.auxdata.iJi = [-(PAR.R*sin(inip(2) - thetai))/(PAR.A*sin(inip(1) - inip(2))),(PAR.R*sin(inip(1) - thetai))/(PAR.B*sin(inip(1) - inip(2))),1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BOUNDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

% setup.bound.lower.phase(cont).initial.position = thetai;
setup.bound.lower.phase(cont).initial.velocity = 0;
setup.bound.upper.phase(cont).initial.velocity = 50*speed;
setup.bound.lower.phase(cont).initial.time = 0;
setup.bound.upper.phase(cont).initial.time = 0;
setup.bound.lower.phase(cont).initial.state = [  0,-inf,0,0,0,0];
setup.bound.upper.phase(cont).initial.state = [inf,+inf,1,1,1,1];
setup.bound.lower.phase(cont).final.velocity = 0;
setup.bound.upper.phase(cont).final.velocity = 50*speed;
setup.bound.lower.phase(cont).final.time = 0.1;
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.lower.phase(cont).final.state = [  0,-inf,0,0,0,0];
setup.bound.upper.phase(cont).final.state = [inf,+inf,1,1,1,1];
setup.bound.lower.phase(cont).position = PAR.thetac1;
setup.bound.upper.phase(cont).position = PAR.thetac2;
setup.bound.lower.phase(cont).velocity = 0;
setup.bound.upper.phase(cont).velocity = 50*speed;
setup.bound.lower.phase(cont).state = [  0,-inf,0,0,0,0];
setup.bound.upper.phase(cont).state = [inf,+inf,1,1,1,1];
setup.bound.lower.phase(cont).control = [0, 0,  30*pi/180,-pi/2, 0, 0,-inf,        0,     0,        0,     0];
setup.bound.upper.phase(cont).control = [1, 1,   maxalpha,   pi, 1, 1,+inf,MShoulder,MElbow,MShoulder,MElbow];
setup.bound.lower.phase(cont).path = [  0*pi/180,0,0,0,-Act_v,-Act_v,-Act_v,-Act_v];
setup.bound.upper.phase(cont).path = [120*pi/180,0,0,0,+Act_v,+Act_v,+Act_v,+Act_v];
cont = cont + 1;

%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%
setup.bound.lower.phase(cont).initial.position = [0,-pi/2,PAR.thetac1];
setup.bound.upper.phase(cont).initial.position = [pi,pi,PAR.thetac2];
setup.bound.lower.phase(cont).initial.velocity = [-10*pi,-10*pi,0];
setup.bound.upper.phase(cont).initial.velocity = [10*pi,10*pi,50*speed];
setup.bound.lower.phase(cont).initial.time = 0.1;
setup.bound.upper.phase(cont).initial.time = inf;
setup.bound.lower.phase(cont).initial.state = [  0,-inf,0,0,0,0];
setup.bound.upper.phase(cont).initial.state = [inf,+inf,1,1,1,1];
setup.bound.lower.phase(cont).final.position = [0,-pi/2,PAR.thetac1];
setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
setup.bound.lower.phase(cont).final.velocity = [-10*pi,-10*pi,0];
setup.bound.upper.phase(cont).final.velocity = [10*pi,10*pi,50*speed];
setup.bound.lower.phase(cont).final.time = 0.2;
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.lower.phase(cont).final.state = [  0,-inf,0,0,0,0];
setup.bound.upper.phase(cont).final.state = [inf,+inf,1,1,1,1];
setup.bound.lower.phase(cont).position = [0,-pi/2,PAR.thetac1];
setup.bound.upper.phase(cont).position = [pi,pi,inf];
setup.bound.lower.phase(cont).velocity = [-10*pi,-10*pi,0];
setup.bound.upper.phase(cont).velocity = [10*pi,10*pi,50*speed];
setup.bound.lower.phase(cont).state = [  0,-inf,0,0,0,0];
setup.bound.upper.phase(cont).state = [inf,+inf,1,1,1,1];
setup.bound.lower.phase(cont).control = [0,0,0,0,0,0,0,0];
setup.bound.upper.phase(cont).control = [1,1,1,1,MShoulder,MElbow,MShoulder,MElbow];
setup.bound.lower.phase(cont).path = [  0*pi/180,-Act_v,-Act_v,-Act_v,-Act_v];
setup.bound.upper.phase(cont).path = [120*pi/180,+Act_v,+Act_v,+Act_v,+Act_v];

pcu = [zeros(1,13+8+4),inf,    inf,+hFT];
pcl = [zeros(1,13+8+4),0.1,Contact,-hFT];

setup.bound.lower.pconstraints = pcl;
setup.bound.upper.pconstraints = pcu;

%% Initial guess
if isnumeric(opt)
    error('InitiaI Guess not defined.');
else
    numl = numel(opt.solution.phase(1).time);
setup.initial_guess.phase(1).time = opt.solution.phase(1).time;
setup.initial_guess.phase(1).position = opt.solution.phase(1).position;
setup.initial_guess.phase(1).velocity = opt.solution.phase(1).velocity;
setup.initial_guess.phase(1).control = [opt.solution.phase(1).control,opt.solution.phase(1).taup];
if size(opt.solution.phase(1).control,2) == 11
	setup.initial_guess.phase(1).control = [opt.solution.phase(1).control];
else
	setup.initial_guess.phase(1).control(:,1) = opt.solution.phase(1).control(:,1)/max(opt.solution.phase(1).control(:,1)+0.01);
	setup.initial_guess.phase(1).control(:,2) = opt.solution.phase(1).control(:,2)/max(opt.solution.phase(1).control(:,2)+0.01);
	setup.initial_guess.phase(1).control(:,3:4) = opt.solution.phase(1).control(:,3:4);
	setup.initial_guess.phase(1).control(:,5) = opt.solution.phase(1).control(:,5)/max(opt.solution.phase(1).control(:,5)+0.01);
	setup.initial_guess.phase(1).control(:,6) = opt.solution.phase(1).control(:,6)/max(opt.solution.phase(1).control(:,6)+0.01);
	setup.initial_guess.phase(1).control(:,7) = opt.solution.phase(1).taup;
	setup.initial_guess.phase(1).control(:,8:9) = opt.solution.phase(1).control(:,1:2);
	setup.initial_guess.phase(1).control(:,10:11) = opt.solution.phase(1).control(:,5:6);   
end
    switch opt.Options.Type
        case 'MSC-PI_Controller'
            setup.initial_guess.phase(1).state(:,1:2) = [opt.solution.phase(1).state];
            setup.initial_guess.phase(1).state(:,3) = opt.solution.phase(1).control(:,1);
            setup.initial_guess.phase(1).state(:,4) = opt.solution.phase(1).control(:,2);
            setup.initial_guess.phase(1).state(:,5) = opt.solution.phase(1).control(:,5);
            setup.initial_guess.phase(1).state(:,6) = opt.solution.phase(1).control(:,6);
        case 'MSC-Reference'
            setup.initial_guess.phase(1).state(:,1:2) = [opt.solution.phase(1).velocity(:,end)*PAR.Rr+rand(numl,1), opt.solution.phase(1).control(:,1)];
            setup.initial_guess.phase(1).state(:,3) = opt.solution.phase(1).control(:,1);
            setup.initial_guess.phase(1).state(:,4) = opt.solution.phase(1).control(:,2);
            setup.initial_guess.phase(1).state(:,5) = opt.solution.phase(1).control(:,5);
            setup.initial_guess.phase(1).state(:,6) = opt.solution.phase(1).control(:,6);
        case 'MSD-Reference'
            setup.initial_guess.phase(1).state(:,1:2) = [opt.solution.phase(1).velocity(:,end)*PAR.Rr+rand(numl,1), opt.solution.phase(1).control(:,1)];
            setup.initial_guess.phase(1).state(:,3:6) = opt.solution.phase(1).state(:,1:4);
        otherwise
            setup.initial_guess.phase(1).state = [opt.solution.phase(1).state];
    end
    
numl = numel(opt.solution.phase(2).time);
setup.initial_guess.phase(2).time = opt.solution.phase(2).time;
setup.initial_guess.phase(2).position = opt.solution.phase(2).position;
setup.initial_guess.phase(2).velocity = opt.solution.phase(2).velocity;
if size(opt.solution.phase(2).control,2) == 8
	setup.initial_guess.phase(2).control = [opt.solution.phase(2).control];
else
	setup.initial_guess.phase(2).control(:,1) = opt.solution.phase(2).control(:,1)/max(opt.solution.phase(2).control(:,1)+0.01);
	setup.initial_guess.phase(2).control(:,2) = opt.solution.phase(2).control(:,2)/max(opt.solution.phase(2).control(:,2)+0.01);
	setup.initial_guess.phase(2).control(:,3) = opt.solution.phase(2).control(:,3)/max(opt.solution.phase(2).control(:,3)+0.01);
	setup.initial_guess.phase(2).control(:,4) = opt.solution.phase(2).control(:,4)/max(opt.solution.phase(2).control(:,4)+0.01);
	setup.initial_guess.phase(2).control(:,5:8) = opt.solution.phase(2).control(:,1:4);   
end
    switch opt.Options.Type
        case 'MSC-PI_Controller'
            setup.initial_guess.phase(2).state(:,1:2) = [opt.solution.phase(2).state];
            setup.initial_guess.phase(2).state(:,3) = opt.solution.phase(2).control(:,1);
            setup.initial_guess.phase(2).state(:,4) = opt.solution.phase(2).control(:,2);
            setup.initial_guess.phase(2).state(:,5) = opt.solution.phase(2).control(:,3);
            setup.initial_guess.phase(2).state(:,6) = opt.solution.phase(2).control(:,4);
        case 'MSC-Reference'
            setup.initial_guess.phase(2).state(:,1:2) = [opt.solution.phase(2).velocity(:,end)*PAR.Rr+rand(numl,1), opt.solution.phase(2).control(:,1)];
            setup.initial_guess.phase(2).state(:,3) = opt.solution.phase(2).control(:,1);
            setup.initial_guess.phase(2).state(:,4) = opt.solution.phase(2).control(:,2);
            setup.initial_guess.phase(2).state(:,5) = opt.solution.phase(2).control(:,3);
            setup.initial_guess.phase(2).state(:,6) = opt.solution.phase(2).control(:,4);
        case 'MSD-Reference'
            setup.initial_guess.phase(2).state(:,1:2) = [opt.solution.phase(2).velocity(:,end)*PAR.Rr+rand(numl,1), opt.solution.phase(2).control(:,1)];
            setup.initial_guess.phase(2).state(:,3:6) = opt.solution.phase(2).state(:,1:4);
        otherwise
            setup.initial_guess.phase(2).state = [opt.solution.phase(2).state];
    end
output = ddiopt_MB(setup);
end

ind = setup.auxdata.indice;
output.solution.phase(1).taup = output.solution.phase(1).control(:,7);
% output.solution.phase(1).control(:,5) = output.solution.phase(1).control(:,6);
% output.solution.phase(1).control(:,6) = output.solution.phase(1).control(:,7);
output.solution.phase(2).taup = zeros(numel(output.solution.phase(2).control(:,1)),1);
output.solution.phase(1).objective = trapz(output.solution.phase(1).time,output.solution.phase(1).control(:,1).^ind+output.solution.phase(1).control(:,5).^ind+ ...
    output.solution.phase(1).control(:,2).^ind+output.solution.phase(1).control(:,6).^ind);
output.solution.phase(2).objective = trapz(output.solution.phase(2).time,output.solution.phase(2).control(:,1).^ind+output.solution.phase(2).control(:,3).^ind+ ...
    output.solution.phase(2).control(:,2).^ind+output.solution.phase(2).control(:,4).^ind);

% Options used in this simulation
output.Options.Type = 'MSD-PI_Controller';
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


function output = Dynamics_guess(input)

% pconstraints = [];
obj = 0;
indice = input.auxdata.indice;
Mi = input.auxdata.PAR.Mi;
Ci = input.auxdata.PAR.Ci;
Kte = input.auxdata.const;
Jtotal = input.auxdata.PAR.Jf; % Jmt - C/Motor; Jmf - S/Motor
Btotal = input.auxdata.PAR.Bf+0.01; % Bt - C/Motor; Bf - S/Motor
tact = 0.02; % 0.01
tdat = 0.04; % 0.04

%%% Original coordinates system: Extension is positive, Flexion is negative
Qv  = cell(1,2);
Qdv = cell(1,2);
act = cell(1,2);
tau = cell(1,2);
dac = cell(1,2);
PHI = cell(1,2);
dPHI= cell(1,2);
TV  = cell(1,2);
TP  = cell(1,2);
CTR = cell(1,2);
LC  = cell(1,2);

%% Propulsion
cnt = 1;

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
as_e  = input.phase(cnt).state(:,3);
as_f  = input.phase(cnt).state(:,5);
ae_e  = input.phase(cnt).state(:,4);
ae_f  = input.phase(cnt).state(:,6);
act{cnt} = [as_e;as_f;ae_e;ae_f];
ts_e  = input.phase(cnt).control(:,8);
ts_f  = input.phase(cnt).control(:,10);
te_e  = input.phase(cnt).control(:,9);
te_f  = input.phase(cnt).control(:,11);
tau{cnt} = [ts_e;ts_f;te_e;te_f];

%% Recovery
cnt = 2;

Qv{cnt}  = input.phase(cnt).position;
Qdv{cnt} = input.phase(cnt).velocity;
	
ds_e  = input.phase(cnt).control(:,1);
ds_f  = input.phase(cnt).control(:,3);
de_e  = input.phase(cnt).control(:,2);
de_f  = input.phase(cnt).control(:,4);
dac{cnt} = [ds_e;ds_f;de_e;de_f];
as_e  = input.phase(cnt).state(:,3);
as_f  = input.phase(cnt).state(:,5);
ae_e  = input.phase(cnt).state(:,4);
ae_f  = input.phase(cnt).state(:,6);
act{cnt} = [as_e;as_f;ae_e;ae_f];
ts_e  = input.phase(cnt).control(:,5);
ts_f  = input.phase(cnt).control(:,7);
te_e  = input.phase(cnt).control(:,6);
te_f  = input.phase(cnt).control(:,8);
tau{cnt} = [ts_e;ts_f;te_e;te_f];

%% Control Law
for i=1:2
    V_imp = input.phase(i).state(:,1);
    i_err = input.phase(i).state(:,2);
    
	err = V_imp/input.auxdata.PAR.Rr-Qdv{i}(3);
    LC{i} = Kte*(Jtotal*err+Btotal*i_err);
    CTR{i} = [V_imp;i_err;err];
end

%% Activation to Torque Constraints
for i=1:2
	[ap1, bt1] = dof_conversion(Qv{i}(1),Qv{i}(2),'Type', 'angle');
	[dap1, dbt1] = dof_conversion(Qdv{i}(1),Qdv{i}(2),'Type', 'angvel');
	[~,~,TV{i},TP{i}] = MTG_smooth(act{i}(2),act{i}(1),act{i}(4),act{i}(3),ap1,bt1,dap1,dbt1);
    TV{i} = 2*TV{i}; TP{i} = 2*TP{i}; % two arms depicted as one
end

%% Propusion
cnt = 1;
	
tau_s1 = tau{cnt}(1)-tau{cnt}(2)-2*TP{cnt}(1);
tau_e1 = tau{cnt}(3)-tau{cnt}(4)-2*TP{cnt}(2);

[M1,ke1,~,H1,k1] = four_bar_system(Qv{cnt}, Qdv{cnt},input.auxdata.PAR,'Type','Reference');
MM1 = PHI{cnt}'*M1*PHI{cnt};
K1  = PHI{cnt}'*M1*dPHI{cnt}*Qdv{cnt}(3);
KE1 = PHI{cnt}'*(H1*[tau_s1;tau_e1]+ke1-k1);
qdd1 = MM1\(KE1-K1);
qdd1 = dPHI{cnt}*Qdv{cnt}(3)+PHI{cnt}*qdd1;
tp = M1(3,:)*qdd1-ke1(3)+k1(3)-LC{cnt};
	
output.phase(cnt).RightHandSide = KE1-K1+LC{cnt};
output.phase(cnt).MassMatrix = MM1;
output.phase(cnt).derivatives = [(1/Mi)*(input.phase(cnt).control(:,7)/input.auxdata.PAR.R-Ci*CTR{cnt}(1)), CTR{cnt}(3), ...
                                 (dac{cnt}(1)-act{cnt}(1))*(dac{cnt}(1)/tact+(1-dac{cnt}(1))/tdat), ...
                                 (dac{cnt}(3)-act{cnt}(3))*(dac{cnt}(3)/tact+(1-dac{cnt}(3))/tdat), ...
                                 (dac{cnt}(2)-act{cnt}(2))*(dac{cnt}(2)/tact+(1-dac{cnt}(2))/tdat), ...
                                 (dac{cnt}(4)-act{cnt}(4))*(dac{cnt}(4)/tact+(1-dac{cnt}(4))/tdat)];
output.phase(cnt).path = [Qv{cnt}(1)-Qv{cnt}(2), ...
	input.auxdata.PAR.A*sin(Qv{cnt}(1))+input.auxdata.PAR.B*sin(Qv{cnt}(2))-input.auxdata.PAR.R*sin(Qv{cnt}(3))-input.auxdata.PAR.Y, ...
	input.auxdata.PAR.A*cos(Qv{cnt}(1))+input.auxdata.PAR.B*cos(Qv{cnt}(2))-input.auxdata.PAR.R*cos(Qv{cnt}(3))-input.auxdata.PAR.h, ...
	tp-input.phase(cnt).control(:,7), ...
    tau{cnt}(2)-TV{cnt}(1), tau{cnt}(1)+TV{cnt}(2), ...
    tau{cnt}(4)-TV{cnt}(3), tau{cnt}(3)+TV{cnt}(4)];

%% Recovery
cnt = 2;
	
tau_s2 = tau{cnt}(1)-tau{cnt}(2)-2*TP{cnt}(1);
tau_e2 = tau{cnt}(3)-tau{cnt}(4)-2*TP{cnt}(2);
	
[M2,ke2,~,H2,k2] = four_bar_system(Qv{cnt}, Qdv{cnt},input.auxdata.PAR,'Type','Reference');
output.phase(cnt).RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2 + [0;0;LC{cnt}];
output.phase(cnt).MassMatrix = M2;
output.phase(cnt).derivatives = [(1/Mi)*(-Ci*CTR{cnt}(1)), CTR{cnt}(3), ...
                                 (dac{cnt}(1)-act{cnt}(1))*(dac{cnt}(1)/tact+(1-dac{cnt}(1))/tdat), ...
                                 (dac{cnt}(3)-act{cnt}(3))*(dac{cnt}(3)/tact+(1-dac{cnt}(3))/tdat), ...
                                 (dac{cnt}(2)-act{cnt}(2))*(dac{cnt}(2)/tact+(1-dac{cnt}(2))/tdat), ...
                                 (dac{cnt}(4)-act{cnt}(4))*(dac{cnt}(4)/tact+(1-dac{cnt}(4))/tdat)];
output.phase(cnt).path = [Qv{cnt}(1)-Qv{cnt}(2), ...
	tau{cnt}(2)-TV{cnt}(1), tau{cnt}(1)+TV{cnt}(2), ...
    tau{cnt}(4)-TV{cnt}(3), tau{cnt}(3)+TV{cnt}(4)];

%% Link Between Cycles

[~,~,PHIi] = idof_transform(input.phase(2).final.position(1),input.phase(2).final.position(2),input.phase(1).initial.position,input.phase(2).final.velocity(3),input.auxdata.PAR);
[~,~,PHIf] = idof_transform(input.phase(2).initial.position(1),input.phase(2).initial.position(2),input.phase(2).initial.position(3),input.phase(2).initial.velocity(3),input.auxdata.PAR);

pconstraints = [input.phase(2).initial.time-input.phase(1).final.time, ...
    input.phase(1).initial.velocity*PHIi'-input.phase(2).final.velocity, ...
	input.auxdata.PAR.A*sin(input.phase(2).final.position(1))+input.auxdata.PAR.B*sin(input.phase(2).final.position(2))-input.auxdata.PAR.R*sin(input.phase(1).initial.position)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(2).final.position(1))+input.auxdata.PAR.B*cos(input.phase(2).final.position(2))-input.auxdata.PAR.R*cos(input.phase(1).initial.position)-input.auxdata.PAR.h, ...
    input.phase(1).final.velocity*PHIf'-input.phase(2).initial.velocity, ...
    input.auxdata.PAR.A*sin(input.phase(2).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(2).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(2).initial.position(3))-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(2).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(2).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(2).initial.position(3))-input.auxdata.PAR.h, ...
    input.phase(2).initial.position(3)-input.phase(1).final.position, ...
    input.phase(2).initial.state-input.phase(1).final.state, ...
    input.phase(1).initial.state-input.phase(2).final.state, ...
    (input.phase(2).final.position(3)-input.phase(1).initial.position)*input.auxdata.PAR.Rr-input.phase(2).final.time*input.auxdata.speed, ...
    input.phase(2).final.time-input.phase(2).initial.time, ...
    input.phase(1).final.position-input.phase(1).initial.position, ...
    col(input.phase(1).control(7),numel(double(input.phase(1).control(7))))];

%% Objective
for i=1:2
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

output.constraints = pconstraints;
output.objective = obj./(input.phase(2).final.time-input.phase(1).initial.time);
% output.objective = obj/(input.phase(2).final.position(3)-input.phase(1).initial.position);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function output = Dynamics_minmax(input)

theta1 = input.phase(1).position;
dtheta1 = input.phase(1).velocity;
tau_s1 = input.phase(1).control(:,1)-input.phase(1).control(:,5);
tau_e1 = input.phase(1).control(:,2)-input.phase(1).control(:,6);
alpha1 = input.phase(1).control(:,3);
beta1 = input.phase(1).control(:,4);
u1 = input.phase(1).control(:,7);
indice = input.auxdata.indice;

[dalpha1,dbeta1,PHI,dPHI] = idof_transform(alpha1,beta1,theta1,dtheta1,input.auxdata.PAR);
q1 = [alpha1,beta1,theta1];
qd1 = [dalpha1,dbeta1,dtheta1];

[M1,ke1,~,H1,k1] = four_bar_system_no_motor(q1, qd1,input.auxdata.PAR);
K = PHI'*M1*dPHI*dtheta1;
output.phase(1).RightHandSide = PHI'*(H1*[tau_s1;tau_e1]+ke1-k1)-K;
output.phase(1).MassMatrix = PHI'*M1*PHI;
KE = PHI'*(H1*[tau_s1;tau_e1]+ke1-k1);
MM = PHI'*M1*PHI;
qdd = (KE-K)./MM;
ddq = dPHI*dtheta1+PHI*qdd;
tp = M1(3,:)*ddq-H1(3,:)*[tau_s1;tau_e1]-ke1(3)+k1(3);
output.phase(1).path = [q1(:,1)-q1(:,2), ...
    input.auxdata.PAR.A*sin(alpha1)+input.auxdata.PAR.B*sin(beta1)-input.auxdata.PAR.R*sin(theta1)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha1)+input.auxdata.PAR.B*cos(beta1)-input.auxdata.PAR.R*cos(theta1)-input.auxdata.PAR.h, ...
    u1-tp,input.phase(1).parameter-(input.phase(1).control(:,1)+input.phase(1).control(:,5)),input.phase(1).parameter-(input.phase(1).control(:,2)+input.phase(1).control(:,6))];
% obj = integrate(input.phase(1).integrand,(input.phase(1).control(:,1)+input.phase(1).control(:,5)).^indice+(input.phase(1).control(:,2)+input.phase(1).control(:,6)).^indice);

q2 = input.phase(2).position;
qd2 = input.phase(2).velocity;
tau_s2 = input.phase(2).control(:,1)-input.phase(2).control(:,3);
tau_e2 = input.phase(2).control(:,2)-input.phase(2).control(:,4);

[M2,ke2,~,H2,k2] = four_bar_system_no_motor(q2, qd2,input.auxdata.PAR);
output.phase(2).RightHandSide = H2*[tau_s2;tau_e2] + ke2 - k2;
output.phase(2).MassMatrix = M2;
output.phase(2).path = [q2(:,1)-q2(:,2), ...
    input.phase(2).parameter-(input.phase(2).control(:,1)+input.phase(2).control(:,3)),input.phase(2).parameter-(input.phase(2).control(:,2)+input.phase(2).control(:,4))];

[~,~,PHIi] = idof_transform(input.phase(2).final.position(1),input.phase(2).final.position(2),input.phase(1).initial.position,input.phase(2).final.velocity(3),input.auxdata.PAR);
[~,~,PHIf] = idof_transform(input.phase(2).initial.position(1),input.phase(2).initial.position(2),input.phase(2).initial.position(3),input.phase(2).initial.velocity(3),input.auxdata.PAR);

pconstraints = [input.phase(2).initial.time-input.phase(1).final.time, ...
    input.phase(1).initial.velocity*PHIi'-input.phase(2).final.velocity, ...
    input.phase(1).final.velocity*PHIf'-input.phase(2).initial.velocity, ...
    input.auxdata.PAR.A*sin(input.phase(2).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(2).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(2).initial.position(3))-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(2).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(2).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(2).initial.position(3))-input.auxdata.PAR.h, ...
    input.phase(2).initial.position(3)-input.phase(1).final.position, ...
    (input.phase(2).final.position(3)-input.phase(1).initial.position)*input.auxdata.PAR.Rr-input.phase(2).final.time*input.auxdata.speed, ...
    input.phase(2).final.time-input.phase(2).initial.time, ...
    input.phase(1).final.position-input.phase(1).initial.position];

% obj = obj+integrate(input.phase(2).integrand,(input.phase(2).control(:,1)+input.phase(2).control(:,3)).^indice+(input.phase(2).control(:,2)+input.phase(2).control(:,4)).^indice);

output.constraints = pconstraints;
output.objective = col(input.phase(1).parameter,1);