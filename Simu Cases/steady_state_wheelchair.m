function output = steady_state_wheelchair(PAR,speed,varargin)

% PAR = parameters;
setup.auxdata.PAR = PAR;
setup.mesh_points = 5;
setup.mesh_number = 20;
setup.function = @Dynamics_guess;
thetai = PAR.thetai;
t_ini = -120*pi/180;

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
parse(p,varargin{:});
optr = p.Results.Guess;
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

if p.Results.MinMax
    setup.function = @Dynamics_minmax;
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
    setup.bound.lower.phase(cont).initial.position = t_ini;
    setup.bound.upper.phase(cont).initial.position = PAR.thetac2;
end

% setup.bound.lower.phase(cont).initial.position = thetai;
setup.bound.lower.phase(cont).initial.velocity = 0;
setup.bound.lower.phase(cont).initial.time = 0;
setup.bound.lower.phase(cont).final.velocity = 0;
setup.bound.lower.phase(cont).final.time = 0.1;
setup.bound.lower.phase(cont).position = PAR.thetac1;
setup.bound.lower.phase(cont).velocity = 0;
setup.bound.lower.phase(cont).control = [0,0,0,-pi/2,0,0,-inf];
setup.bound.lower.phase(cont).path = [10*pi/180,0,0,0,0,0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup.bound.upper.phase(cont).initial.position = thetai;
setup.bound.upper.phase(cont).initial.velocity = 50*speed;
setup.bound.upper.phase(cont).initial.time = 0;
setup.bound.upper.phase(cont).final.velocity = 50*speed;
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.upper.phase(cont).position = PAR.thetac2;
setup.bound.upper.phase(cont).velocity = 50*speed;
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,pi,pi,MShoulder,MElbow,inf];
setup.bound.upper.phase(cont).path = [pi,0,0,0,inf,inf];
cont = cont + 1;
%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%
setup.bound.lower.phase(cont).initial.position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).initial.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).initial.time = 0.1;
% setup.bound.lower.phase(2).final.position = [0,-pi/2,-150*pi/180];
setup.bound.lower.phase(cont).final.position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).final.velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).final.time = 0.2;
setup.bound.lower.phase(cont).position = [0,-pi/2,PAR.thetac1];
setup.bound.lower.phase(cont).velocity = [-10*pi,-10*pi,0];
setup.bound.lower.phase(cont).control = [0,0,0,0];
setup.bound.lower.phase(cont).path = [0,0,0];
setup.bound.upper.phase(cont).initial.position = [pi,pi,PAR.thetac2];
setup.bound.upper.phase(cont).initial.velocity = [10*pi,10*pi,50*speed];
setup.bound.upper.phase(cont).initial.time = inf;
setup.bound.upper.phase(cont).final.position = [pi,pi,inf];
setup.bound.upper.phase(cont).final.velocity = [10*pi,10*pi,50*speed];
setup.bound.upper.phase(cont).final.time = inf;
setup.bound.upper.phase(cont).position = [pi,pi,inf];
setup.bound.upper.phase(cont).velocity = [10*pi,10*pi,50*speed];
setup.bound.upper.phase(cont).control = [MShoulder,MElbow,MShoulder,MElbow];
setup.bound.upper.phase(cont).path = [pi,inf,inf];
setup.bound.lower.parameter = 0;
setup.bound.upper.parameter = inf;
pcu = [zeros(1,13),inf,inf];
pcl = [zeros(1,13),0.1,Contact];

setup.bound.lower.pconstraints = pcl;
setup.bound.upper.pconstraints = pcu;

%% Initial guess
if isnumeric(optr)
    load('wheelchair_ni40_Froll15_nn40_v05.mat')
setup.initial_guess.phase(1).time = opt.t1;
setup.initial_guess.phase(1).position = opt.x1/PAR.Rr+thetai;
setup.initial_guess.phase(1).velocity = opt.xd1/PAR.Rr;
setup.initial_guess.phase(1).control = [opt.tau_s1,opt.tau_e1,pi/2-opt.beta1,pi/2-opt.alpha1,-opt.tau_s1,-opt.tau_e1,opt.tau_s1];
setup.initial_guess.phase(2).time = opt.t2;
setup.initial_guess.phase(2).position = [pi/2-opt.beta2,pi/2-opt.alpha2,opt.x2/PAR.Rr+thetai];
setup.initial_guess.phase(2).velocity = [-opt.betad2,-opt.alphad2,opt.xd2/PAR.Rr];
setup.initial_guess.phase(2).control = [opt.tau_s2,opt.tau_e2,-opt.tau_s2,-opt.tau_e2];
setup.initial_guess.parameter = 100;
output = ddiopt_MB(setup);
else
setup.initial_guess.phase(1).time = optr.solution.phase(1).time;
setup.initial_guess.phase(1).position = optr.solution.phase(1).position;
setup.initial_guess.phase(1).velocity = optr.solution.phase(1).velocity;
setup.initial_guess.phase(1).control = [optr.solution.phase(1).control,optr.solution.phase(1).taup];
setup.initial_guess.phase(2).time = optr.solution.phase(2).time;
setup.initial_guess.phase(2).position = optr.solution.phase(2).position;
setup.initial_guess.phase(2).velocity = optr.solution.phase(2).velocity;
setup.initial_guess.phase(2).control = [optr.solution.phase(2).control,optr.solution.phase(2).taup];
setup.initial_guess.parameter =  max(max(optr.solution.phase(2).control));
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
output.Options.Type = 'Reference';
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


function output = Dynamics_guess(input)

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
obj = integrate(input.phase(1).integrand,(input.phase(1).control(:,1)).^indice+(input.phase(1).control(:,2)).^indice);
obj = obj + integrate(input.phase(1).integrand,(input.phase(1).control(:,5)).^indice+(input.phase(1).control(:,6)).^indice);
% obj = integrate(input.phase(1).integrand,(input.phase(1).control(:,1)).^indice+(input.phase(1).control(:,2)).^indice)./(input.phase(1).final.time-input.phase(1).initial.time);
% obj = obj + integrate(input.phase(1).integrand,(input.phase(1).control(:,5)).^indice+(input.phase(1).control(:,6)).^indice)./(input.phase(1).final.time-input.phase(1).initial.time);
% obj = obj + 5*integrate(input.phase(1).integrand,dalpha1*tau_s1+dbeta1*tau_e1)*10^1;

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
	input.auxdata.PAR.A*sin(input.phase(2).final.position(1))+input.auxdata.PAR.B*sin(input.phase(2).final.position(2))-input.auxdata.PAR.R*sin(input.phase(1).initial.position)-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(2).final.position(1))+input.auxdata.PAR.B*cos(input.phase(2).final.position(2))-input.auxdata.PAR.R*cos(input.phase(1).initial.position)-input.auxdata.PAR.h, ...
    input.phase(1).final.velocity*PHIf'-input.phase(2).initial.velocity, ...
    input.auxdata.PAR.A*sin(input.phase(2).initial.position(1))+input.auxdata.PAR.B*sin(input.phase(2).initial.position(2))-input.auxdata.PAR.R*sin(input.phase(2).initial.position(3))-input.auxdata.PAR.Y, ...
    input.auxdata.PAR.A*cos(input.phase(2).initial.position(1))+input.auxdata.PAR.B*cos(input.phase(2).initial.position(2))-input.auxdata.PAR.R*cos(input.phase(2).initial.position(3))-input.auxdata.PAR.h, ...
    input.phase(2).initial.position(3)-input.phase(1).final.position, ...
    (input.phase(2).final.position(3)-input.phase(1).initial.position)*input.auxdata.PAR.Rr-input.phase(2).final.time*input.auxdata.speed, ...
    input.phase(2).final.time-input.phase(2).initial.time, ...
    input.phase(1).final.position-input.phase(1).initial.position];

obj = obj+integrate(input.phase(2).integrand,(input.phase(2).control(:,1)).^indice+(input.phase(2).control(:,2)).^indice);
obj = obj+integrate(input.phase(2).integrand,(input.phase(2).control(:,3)).^indice+(input.phase(2).control(:,4)).^indice);
% obj = obj+integrate(input.phase(2).integrand,(input.phase(2).control(:,1)).^indice+(input.phase(2).control(:,2)).^indice)./(input.phase(2).final.time-input.phase(2).initial.time);
% obj = obj+integrate(input.phase(2).integrand,(input.phase(2).control(:,3)).^indice+(input.phase(2).control(:,4)).^indice)./(input.phase(2).final.time-input.phase(2).initial.time);
% obj = obj + 5*integrate(input.phase(2).integrand,qd2(1)*tau_s2+qd2(2)*tau_e2)*10^1;

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