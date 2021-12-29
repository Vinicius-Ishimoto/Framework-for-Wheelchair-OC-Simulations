function output =  multi_cycle_wheelchair(PAR,ncycle,velocity,distance,varargin)


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

cont = 1;
ub = [];
lb = [];
for q = 1:ncycle
if cont==1
    setup.bound.lower.phase(cont).initial.velocity = 0;
    setup.bound.upper.phase(cont).initial.velocity = 0;
    setup.bound.lower.phase(cont).initial.time = 0;
    setup.bound.upper.phase(cont).initial.time = 0;
else
    setup.bound.lower.phase(cont).initial.velocity = 0;
    setup.bound.upper.phase(cont).initial.velocity = 50;
    setup.bound.lower.phase(cont).initial.time = 0;
    setup.bound.upper.phase(cont).initial.time = inf;
end
if not(isFixed)
    setup.bound.lower.phase(cont).initial.position = t_ini;
    setup.bound.lower.phase(cont).final.position = PAR.thetac1;
    setup.bound.upper.phase(cont).initial.position = PAR.thetac2;
    setup.bound.upper.phase(cont).final.position = PAR.thetac2;    
    
else
    setup.bound.lower.phase(cont).initial.position = PAR.thetai;
    setup.bound.lower.phase(cont).final.position = PAR.thetac1;
    setup.bound.upper.phase(cont).initial.position = PAR.thetai;
    setup.bound.upper.phase(cont).final.position = PAR.thetac2;
end

setup.bound.lower.phase(cont).final.velocity = 0;
setup.bound.lower.phase(cont).final.time = 0.1;
setup.bound.lower.phase(cont).position = PAR.thetac1;
setup.bound.lower.phase(cont).velocity = 0;
setup.bound.lower.phase(cont).control = [0,0,0,-pi/2,0,0];
setup.bound.lower.phase(cont).path = [0,0,0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.upper.phase(cont).final.velocity = 50;
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.upper.phase(cont).position = PAR.thetac2;
setup.bound.upper.phase(cont).velocity = 50;
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,pi,pi,MShoulder,MElbow];
setup.bound.upper.phase(cont).path = [pi,0,0];
setup.phase(cont).mesh_points = 10;
setup.phase(cont).mesh_number = 10;
cont = cont + 1;

setup.bound.lower.phase(cont).initial.position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).initial.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).initial.time = 0.1;
% setup.bound.lower.phase(cont).final.position = [0,-pi/2,-150*pi/180];
setup.bound.lower.phase(cont).final.position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).final.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).final.time = 0.2;
setup.bound.lower.phase(cont).position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).control = [0,0,0,0];
setup.bound.lower.phase(cont).path = 0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.upper.phase(cont).initial.position = [pi,pi,PAR.thetac2];
setup.bound.upper.phase(cont).initial.velocity = [10*pi,10*pi,50];
setup.bound.upper.phase(cont).initial.time = inf;
% setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
setup.bound.upper.phase(cont).final.velocity = [10*pi,10*pi,50];
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.upper.phase(cont).position = [pi,pi,inf];
setup.bound.upper.phase(cont).velocity = [10*pi,10*pi,50];
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,MShoulder,MElbow];
setup.bound.upper.phase(cont).path = pi;
if q==ncycle
    setup.phase(cont).mesh_points = 10;
    setup.phase(cont).mesh_number = 10;
else
    setup.phase(cont).mesh_points = 12;
    setup.phase(cont).mesh_number = 12;
end
if hasFixedStart
	ub = [ub,zeros(1,13),inf,inf,    inf,0];
	lb = [lb,zeros(1,13),mintime,mintime,Contact,0];
else
	ub = [ub,zeros(1,13),inf,inf,    inf,inf];
	lb = [lb,zeros(1,13),mintime,mintime,Contact,-inf];
end
cont = cont + 1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.lower.phase(cont).initial.position = t_ini;
setup.bound.lower.phase(cont).initial.velocity = 0;
setup.bound.lower.phase(cont).initial.time = 0;
setup.bound.lower.phase(cont).final.position = PAR.thetac1;
setup.bound.lower.phase(cont).final.velocity = 0;

setup.bound.lower.phase(cont).position = PAR.thetac1;
setup.bound.lower.phase(cont).velocity = 0;
setup.bound.lower.phase(cont).control = [0,0,0,-pi/2,0,0];
setup.bound.lower.phase(cont).path = [0,0,0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
setup.bound.upper.phase(cont).initial.position = PAR.thetac2;
setup.bound.upper.phase(cont).initial.velocity = 50;
setup.bound.upper.phase(cont).initial.time = inf;
setup.bound.upper.phase(cont).final.position = PAR.thetac2;
setup.bound.upper.phase(cont).final.velocity = inf;
if FixedDist
    setup.bound.lower.phase(cont).final.time = distance/velocity;
    setup.bound.upper.phase(cont).final.time = distance/velocity;
else
    setup.bound.lower.phase(cont).final.time = 0.2;
    setup.bound.upper.phase(cont).final.time = inf;
end
setup.bound.upper.phase(cont).position = PAR.thetac2;
setup.bound.upper.phase(cont).velocity = 50;
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,pi,pi,MShoulder,MElbow];
setup.bound.upper.phase(cont).path = [pi,0,0];
if hasFixedStart
	setup.bound.lower.pconstraints = [lb,  0.1,velocity,  1,0];
	setup.bound.upper.pconstraints = [ub,  inf,velocity,inf,0];
else
	setup.bound.lower.pconstraints = [lb,  0.1,velocity,  1,-inf];
	setup.bound.upper.pconstraints = [ub,  inf,velocity,inf,inf];
end
setup.phase(cont).mesh_points = 8;
setup.phase(cont).mesh_number = 10;

setup.bound.lower.parameter = [PAR.thetac1];
setup.bound.upper.parameter = [PAR.thetac2];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIAL GUESS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if isnumeric(opt)
	ini = 'wheelchair_ni40_Froll15_nn40_v05.mat';
	load(ini)
	cont = 1;
	for q=1:ncycle
		setup.initial_guess.phase(cont).time = opt.t1;
		setup.initial_guess.phase(cont).position = opt.x1/PAR.Rr+PAR.thetai;
		setup.initial_guess.phase(cont).velocity = opt.xd1/PAR.Rr;
		% setup.initial_guess.phase(1).control = [-opt.tau_s1,-opt.tau_e1,-opt.alpha1,-opt.beta1];
		setup.initial_guess.phase(cont).control = [-opt.tau_s1,-opt.tau_e1,pi/2-opt.alpha1,pi/2-opt.beta1,opt.tau_s1,opt.tau_e1];
		cont = cont + 1;
		
		setup.initial_guess.phase(cont).time = opt.t2;
		setup.initial_guess.phase(cont).position = [pi/2-opt.alpha2,pi/2-opt.beta2,opt.x2/PAR.Rr+PAR.thetai];
		setup.initial_guess.phase(cont).velocity = [-opt.alphad2,-opt.betad2,opt.xd2/PAR.Rr];
		setup.initial_guess.phase(cont).control = -[opt.tau_s2,opt.tau_e2,-opt.tau_s2,-opt.tau_e2];
		cont = cont + 1;
	end
	setup.initial_guess.phase(cont).time = opt.t1+opt.t2(end);
	setup.initial_guess.phase(cont).position = opt.x1/PAR.Rr+PAR.thetai;
	setup.initial_guess.phase(cont).velocity = opt.xd1/PAR.Rr;
	% setup.initial_guess.phase(1).control = [-opt.tau_s1,-opt.tau_e1,-opt.alpha1,-opt.beta1];
	setup.initial_guess.phase(cont).control = [-opt.tau_s1,-opt.tau_e1,pi/2-opt.alpha1,pi/2-opt.beta1,opt.tau_s1,opt.tau_e1];
	setup.initial_guess.parameter = -100*pi/180;
	output = ddiopt_MB(setup);
else
	cont = 1;
	for q = 1:ncycle
		setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
		setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
		setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
		if size(opt.solution.phase(cont).control,2) == 6
			setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control];
		else
			setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control,-opt.solution.phase(cont).control(:,1:2)];    
		end
		cont = cont + 1;
		
		setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
		setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
		setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
		if size(opt.solution.phase(cont).control,2) == 4
			setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control];
		else
			setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control,-opt.solution.phase(cont).control(:,1:2)];    
		end
		cont = cont + 1;
	end
	setup.initial_guess.phase(cont).time = opt.solution.phase(cont).time;
	setup.initial_guess.phase(cont).position = opt.solution.phase(cont).position;
	setup.initial_guess.phase(cont).velocity = opt.solution.phase(cont).velocity;
	if size(opt.solution.phase(cont).control,2) == 6
		setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control];
	else
		setup.initial_guess.phase(cont).control = [opt.solution.phase(cont).control,-opt.solution.phase(cont).control(:,1:2)];    
	end
	setup.initial_guess.parameter = -100*pi/180;
	output = ddiopt_MB(setup);
end

cont = 1;
for q = 1:ncycle
    q1 = vect([output.solution.phase(cont).control(:,3:4),output.solution.phase(cont).position]);
    [da,db,PHI,dPHI] = idof_transform(vect(output.solution.phase(cont).control(:,3)),vect(output.solution.phase(cont).control(:,4)),vect(output.solution.phase(cont).position),vect(output.solution.phase(cont).velocity),PAR);
    qd1 = [da,db,vect(output.solution.phase(cont).velocity)];
    [M1,ke1,~,H1,k1] = four_bar_system_no_motor(q1, qd1, PAR);
    
    K = PHI'*(ke1+H1*[vect(output.solution.phase(cont).control(:,1))-vect(output.solution.phase(cont).control(:,5));vect(output.solution.phase(cont).control(:,2))-vect(output.solution.phase(cont).control(:,6))]-k1);
    MM = PHI'*M1*PHI;
    ddt = (K-PHI'*M1*dPHI*vect(output.solution.phase(cont).velocity))./MM;
    qdd = PHI*ddt+dPHI*vect(output.solution.phase(cont).velocity);
    tp = M1(3,:)*qdd-(ke1(3)+H1(3,:)*[vect(output.solution.phase(cont).control(:,1))-vect(output.solution.phase(cont).control(:,5));vect(output.solution.phase(cont).control(:,2))-vect(output.solution.phase(cont).control(:,6))]-k1(3));
    output.solution.phase(cont).taup = double(tp);
    cont = cont + 1;
    
    output.solution.phase(cont).taup = zeros(numel(output.solution.phase(cont).velocity(:,1)),1);
    cont = cont + 1;
end
    q1 = vect([output.solution.phase(cont).control(:,3:4),output.solution.phase(cont).position]);
    [da,db,PHI,dPHI] = idof_transform(vect(output.solution.phase(cont).control(:,3)),vect(output.solution.phase(cont).control(:,4)),vect(output.solution.phase(cont).position),vect(output.solution.phase(cont).velocity),PAR);
    qd1 = [da,db,vect(output.solution.phase(cont).velocity)];
    [M1,ke1,~,H1,k1] = four_bar_system_no_motor(q1, qd1, PAR);
    
    K = PHI'*(ke1+H1*[vect(output.solution.phase(cont).control(:,1))-vect(output.solution.phase(cont).control(:,5));vect(output.solution.phase(cont).control(:,2))-vect(output.solution.phase(cont).control(:,6))]-k1);
    MM = PHI'*M1*PHI;
    ddt = (K-PHI'*M1*dPHI*vect(output.solution.phase(cont).velocity))./MM;
    qdd = PHI*ddt+dPHI*vect(output.solution.phase(cont).velocity);
    tp = M1(3,:)*qdd-(ke1(3)+H1(3,:)*[vect(output.solution.phase(cont).control(:,1))-vect(output.solution.phase(cont).control(:,5));vect(output.solution.phase(cont).control(:,2))-vect(output.solution.phase(cont).control(:,6))]-k1(3));
    output.solution.phase(cont).taup = double(tp);    
	
% Options used in this simulation
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

function output = Dynamics_Multi(input)

p_constraints = [];
obj = 0;
theta_total = 0;
cont = 1;
indice = input.auxdata.indice;

for q=1:input.auxdata.ncycle
theta1 = input.phase(cont).position;
dtheta1 = input.phase(cont).velocity;
tau_s1 = input.phase(cont).control(:,1)-input.phase(cont).control(:,5);
tau_e1 = input.phase(cont).control(:,2)-input.phase(cont).control(:,6);
alpha1 = input.phase(cont).control(:,3);
beta1 = input.phase(cont).control(:,4);
theta_initial = col(input.phase(cont).parameter,1);

[dalpha1,dbeta1,PHI,dPHI] = idof_transform(alpha1,beta1,theta1,dtheta1,input.auxdata.PAR);
q1 = [alpha1,beta1,theta1];
qd1 = [dalpha1,dbeta1,dtheta1];

[M1,ke1,~,H1,k1] = four_bar_system_no_motor(q1, qd1,input.auxdata.PAR);
K = PHI'*M1*dPHI*dtheta1;
output.phase(cont).RightHandSide = PHI'*(H1*[tau_s1;tau_e1]+ke1-k1)-K;
output.phase(cont).MassMatrix = PHI'*M1*PHI;
output.phase(cont).path = [q1(:,1)-q1(:,2), ...
    input.auxdata.PAR.A*sin(alpha1)+input.auxdata.PAR.B*sin(beta1)-input.auxdata.PAR.R*sin(theta1)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha1)+input.auxdata.PAR.B*cos(beta1)-input.auxdata.PAR.R*cos(theta1)-input.auxdata.PAR.h];

% dtime = input.phase(cont+1).final.time-input.phase(cont).initial.time;
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);
cont = cont + 1;

q2 = input.phase(cont).position;
qd2 = input.phase(cont).velocity;
tau_s2 = input.phase(cont).control(:,1)-input.phase(cont).control(:,3);
tau_e2 = input.phase(cont).control(:,2)-input.phase(cont).control(:,4);

[M2,ke2,~,H2,k2] = four_bar_system_no_motor(q2, qd2,input.auxdata.PAR);
output.phase(cont).RightHandSide = H2*[tau_s2;tau_e2] + ke2 -k2;
output.phase(cont).MassMatrix = M2;
output.phase(cont).path = q2(:,1)-q2(:,2);

[~,~,PHI2i] = idof_transform(input.phase(cont).initial.position(1),input.phase(cont).initial.position(2),input.phase(cont).initial.position(3),input.phase(cont).initial.velocity(3),input.auxdata.PAR);
[~,~,PHI2f] = idof_transform(input.phase(cont).final.position(1),input.phase(cont).final.position(2),input.phase(cont+1).initial.position,input.phase(cont).final.velocity(3),input.auxdata.PAR);
p_constraints = [p_constraints, input.phase(cont).initial.time-input.phase(cont-1).final.time];
p_constraints = [p_constraints,input.phase(cont).initial.position(3)-input.phase(cont-1).final.position, ...
    input.auxdata.PAR.A*sin(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(cont).initial.position(3))-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(cont).initial.position(3))-input.auxdata.PAR.h, ...
    input.phase(cont-1).final.velocity*PHI2i'-input.phase(cont).initial.velocity, ...
    input.auxdata.PAR.A*sin(input.phase(cont).final.position(1))+input.auxdata.PAR.B*sin(input.phase(cont).final.position(2))-input.auxdata.PAR.R*sin(input.phase(cont+1).initial.position)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(cont).final.position(1))+input.auxdata.PAR.B*cos(input.phase(cont).final.position(2))-input.auxdata.PAR.R*cos(input.phase(cont+1).initial.position)-input.auxdata.PAR.h, ...
    input.phase(cont).final.velocity-PHI2f'*input.phase(cont+1).initial.velocity, ...
    input.phase(cont+1).initial.time-input.phase(cont).final.time, ...
    input.phase(cont-1).final.time-input.phase(cont-1).initial.time, ...
    input.phase(cont).final.time-input.phase(cont).initial.time, ...
    input.phase(cont-1).final.position-input.phase(cont-1).initial.position, ...
	theta_initial - input.phase(cont-1).initial.position];
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,3).^indice+input.phase(cont).control(:,4).^indice);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice)./(input.phase(cont).final.position(3)-input.phase(cont).initial.position(3));
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,3).^indice+input.phase(cont).control(:,4).^indice)./(input.phase(cont).final.position(3)-input.phase(cont).initial.position(3));
theta_total = theta_total + input.phase(cont).final.position(3)-input.phase(cont-1).initial.position;
cont = cont + 1;
end
theta1 = input.phase(cont).position;
dtheta1 = input.phase(cont).velocity;
tau_s1 = input.phase(cont).control(:,1)-input.phase(cont).control(:,5);
tau_e1 = input.phase(cont).control(:,2)-input.phase(cont).control(:,6);
alpha1 = input.phase(cont).control(:,3);
beta1 = input.phase(cont).control(:,4);
theta_initial = col(input.phase(cont).parameter,1);

[dalpha1,dbeta1,PHI,dPHI] = idof_transform(alpha1,beta1,theta1,dtheta1,input.auxdata.PAR);
q1 = [alpha1,beta1,theta1];
qd1 = [dalpha1,dbeta1,dtheta1];

[M1,ke1,~,H1,k1] = four_bar_system_no_motor(q1, qd1,input.auxdata.PAR);
K = PHI'*M1*dPHI*dtheta1;
output.phase(cont).RightHandSide = PHI'*(H1*[tau_s1;tau_e1]+ke1-k1)-K;
output.phase(cont).MassMatrix = PHI'*M1*PHI;
output.phase(cont).path = [q1(:,1)-q1(:,2), ...
    input.auxdata.PAR.A*sin(alpha1)+input.auxdata.PAR.B*sin(beta1)-input.auxdata.PAR.R*sin(theta1)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(alpha1)+input.auxdata.PAR.B*cos(beta1)-input.auxdata.PAR.R*cos(theta1)-input.auxdata.PAR.h];

% dtime = input.phase(cont).final.time-input.phase(cont).initial.time;
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice);
obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,1).^indice+input.phase(cont).control(:,2).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);
% obj = obj + integrate(input.phase(cont).integrand,input.phase(cont).control(:,5).^indice+input.phase(cont).control(:,6).^indice)./(input.phase(cont).final.position-input.phase(cont).initial.position);

theta_total = (theta_total+input.phase(cont).final.position-input.phase(cont).initial.position);
mean_velocity = theta_total*input.auxdata.PAR.Rr./input.phase(cont).final.time;
p_constraints = [p_constraints, input.phase(cont).final.time-input.phase(cont).initial.time, ...
    mean_velocity, ...
    obj, theta_initial - input.phase(cont).initial.position];

%      ...


output.constraints = p_constraints;
% output.constraints = [input.phase(2).initial.time-input.phase(1).final.time,input.phase(2).final.time-input.phase(2).initial.time, ...
%     input.phase(1).final.position(3)-input.phase(1).final.position(2),input.phase(2).final.position(3)-input.phase(2).final.position(2)];
output.objective = obj./(input.phase(cont).final.time-input.phase(cont).initial.time);
% output.objective = obj;