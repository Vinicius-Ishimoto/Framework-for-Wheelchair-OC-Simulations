% Simulações Com controle - Angulo de rampa variável

addpath (genpath ('C:\Users\viniciusi.c\Downloads\simu'));
addpath(genpath ('Z:\Controle �timo'));

%% Person data
Person.Height = 1.7;
Person.Mass = 70;
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass);
Max_Shoulder = 124; % Maximum torque in shoulder
Max_Elbow = 90; % Maximum torque in elbow
Indice = 2;
% Roling Resistance
Simulations = [0; 2; 4; 6];
% Simulations = Simulations.*pi./180;

%% Simulation data (1) Steady-State
Speed = 0.9;
Flag = 2; % 1 == fixed contact angle; 2 == free contact angle
Contact = 50;
Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice};
Results.Reference = steady_state_wheelchair(Person.Reference, Speed, Options{:});

%% Data Analysis
DATA = fvalid(Person.Reference, Results.Reference);

% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*Person.Reference.Rr;
u  = interp1(DATA.time,DATA.taup/Person.Reference.R,t,'linear','extrap');

reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*Person.Reference.Rr,'-r','linewidth',2)
	plot(T,Y+mean(dx-Y),'-b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

Graphics(Person.Reference,Results.Reference,'Reference');

for q=1:size(Simulations,1)

	%% Simulation data
    Person.Controller{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle', Simulations(q), ...
        'Mi', ReferenceModel.Mass, 'Ci', ReferenceModel.Friction, ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);
    
	Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
	Guess = multi_cycle_PID_integrate(Person.Controller{q}, Results.Reference);
    
	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess, 'indice', Indice};
    Results.References{q} = steady_state_wheelchair(Person.Controller{q}, Speed, Options{:});
	Results.Controller{q} = steady_state_PID_controller(Person.Controller{q}, Speed, Options{:});
	
	Graphics(Person.Controller{q},Results.Controller{q},['Controller ',int2str(q),'.']);

end

save([cd,'\Results\','byAngle_Indice',int2str(Indice),'_SteadyState','.mat'],'Results','Person','ReferenceModel','Options','Simulations')

%% Simulation data (2) Energetic Race
clear Results

guess = load('non_linearMC09.mat');
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass);
% Indice = 2;
Speed = 0.9;
Num_Cycles = 2;
Distance = 2;
Flag = 2; % 1 == fixed contact angle; 2 == free contact angle
Contact = 10; % Minimum contact angle
Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', guess.resp,'indice',Indice};
Results.Reference = multi_cycle_wheelchair(Person.Reference, Num_Cycles, Speed, Distance, Options{:});

%% Data Analysis
DATA = fvalid(Person.Reference, Results.Reference);

% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*Person.Reference.Rr;
u  = interp1(DATA.time,DATA.taup/Person.Reference.R,t,'linear','extrap');

reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*Person.Reference.Rr,'-r','linewidth',2)
	plot(T,Y+mean(dx-Y),'-b','linewidth',2)
hold off
legend('d\theta Sim',['d\theta Ref',char(10),sprintf('M_i = %0.2f',ReferenceModel.Mass),char(10),sprintf('C_i = %0.2f',ReferenceModel.Friction)],'Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

Graphics(Person.Reference,Results.Reference,'Reference');

%% Simulations with controller

for q=1:size(Simulations,1)

	%% Simulation data
    Person.Controller{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle', Simulations(q), ...
        'Mi', ReferenceModel.Mass, 'Ci', ReferenceModel.Friction, ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);
    
	Speed = Results.Reference.Options.MeanVelocity;
	Num_Cycles = Results.Reference.Options.NumCycles;
	Distance = Results.Reference.Options.TotalDistance;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
	Guess = multi_cycle_PID_integrate(Person.Controller{q}, Results.Reference);
	%%
	Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', 10, 'Guess', Guess,'indice',Results.Reference.Options.Indice};
    Results.References{q} = multi_cycle_wheelchair(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
	Results.Controller{q} = multi_cycle_PID_controller(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
	
	Graphics(Person.Controller{q},Results.Controller{q},['Controller ',int2str(q),'.']);

end

save([cd,'\Results\','byAngle_Indice',int2str(Indice),'_EnergeticRace','.mat'],'Results','Person','ReferenceModel','Options','Simulations')