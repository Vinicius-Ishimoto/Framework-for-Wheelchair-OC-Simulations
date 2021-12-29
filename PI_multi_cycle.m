% SimulaÃ§Ãµes Com controle variado

addpath (genpath ('C:\Users\viniciusi.c\Downloads\simu'));
addpath(genpath ('Z:\Controle Ótimo'));
addpath(genpath('C:\Users\viniciusi.c\Downloads\mexIPOPT-1.1.3'));

%% Person data
Person.Height = 1.7;
Person.Mass = 70;
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass);
Max_Shoulder = 124; % Maximum torque in shoulder
Max_Elbow = 90; % Maximum torque in elbow
Indice = 2;
AngRamp = 3;
ATt = true;
% Mass, Friction
Simulations = [1, 1;
               0.5, 1;
               1, 0.5;
			   0.5, 0.5];
           
%% Simulation data (2) Energetic Race
% clear all;
Guess = load('non_linearMC09.mat');
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass);
% Indice = 2;
Speed = 0.9;
Num_Cycles = 2;
Distance = 1.6;
Method = 'tfest';  % mq || tfest
Flag = 2; % 1 == fixed contact angle; 2 == free contact angle
Contact = 30; % Minimum contact angle
Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', Guess.resp,'indice',Indice, 'FixedStart', true};
Guess.NoDyn = multi_cycle_wheelchair_iii(Person.Reference, Num_Cycles, Speed, Distance, Options{:});

Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', Guess.NoDyn,'indice',Indice, 'FixedStart', true};
Guess.WtDyn = MP_muscle_wheelchair(Person.Reference,Num_Cycles, Speed, Distance,Options{:});

Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', Guess.WtDyn,'indice',Indice, 'FixedEndTt', ATt};
Results.Reference = MP_muscledyn_wheelchair(Person.Reference,Num_Cycles, Speed, Distance,Options{:});

%% Data Analysis
DATA = fvalid(Person.Reference, Results.Reference);

t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*(Person.Reference.Rr);
u  = interp1(DATA.time,DATA.taup/Person.Reference.Rr,t,'linear','extrap');

switch Method
    case 'mq'
        % Least Square
        reg = [dx(1:end-1),u(1:end-1)];
        thetaf = pinv(reg)*dx(2:end);

        ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
        ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);
        y0 = mean(dx-Y);
    case 'tfest'
        % tfest
        data = iddata(dx,u,t(2)-t(1));
        data.InputName  = 'Força';
        data.InputUnit  = 'N';
        data.OutputName = 'Velocidade';
        data.OutputUnit = 'm/s';
        data.TimeUnit   = 's';

        opt = tfestOptions('InitialCondition','zero');
        SYS = tfest(data,1,0,opt);
        [NUM,DEN,TS] = tfdata(SYS);
        % S = SYS;
        ReferenceModel.Mass = 1/NUM{1}(end);
        ReferenceModel.Friction = DEN{1}(end)/NUM{1}(end);
        y0 = 0;
end

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*Person.Reference.Rr,'-r','linewidth',2)
	plot(T,Y+y0,'-b','linewidth',2)
hold off
legend('d\theta Sim',['d\theta Ref',char(10),sprintf('M_i = %0.2f',ReferenceModel.Mass),char(10),sprintf('C_i = %0.2f',ReferenceModel.Friction)],'Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

FDATA.Plano.Reference.Person = Person.Reference;
FDATA.Plano.Reference.Options = Options;
FDATA.Plano.Reference.Results = fvalid(Person.Reference,Results.Reference);
FDATA.Plano.Reference.Raw = Results.Reference;
FDATA.ReferenceModel = ReferenceModel;
Graphics(Person.Reference,Results.Reference,'Reference');

% Guesses1 = load('byAssistiveController_Indice2_EnergeticRace_c2.mat');

%%
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle', AngRamp);
Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', Guess.WtDyn,'indice',Indice, 'FixedStart', true};
Guess.WtDyn = MP_muscle_wheelchair(Person.Reference,Num_Cycles, Speed, Distance,Options{:});

Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', Guess.WtDyn,'indice',Indice, 'FixedEndTt', ATt};
Results.Reference = MP_muscledyn_wheelchair(Person.Reference,Num_Cycles, Speed, Distance,Options{:});

FDATA.Rampa.Reference.Person = Person.Reference;
FDATA.Rampa.Reference.Options = Options;
FDATA.Rampa.Reference.Results = fvalid(Person.Reference,Results.Reference);
FDATA.Rampa.Reference.Raw = Results.Reference;
FDATA.ReferenceModel = ReferenceModel;
Graphics(Person.Reference,Results.Reference,'Reference - Rampa');

% Guesses2 = load('byAssistiveController_Indice2_EnergeticRace_c2.mat');

%%

for q=1:size(Simulations,1)

	%% Simulation data
    Person.Controller{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);
	Speed = Results.Reference.Options.MeanVelocity;
	Num_Cycles = Results.Reference.Options.NumCycles;
	Distance = Results.Reference.Options.TotalDistance;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow

	%%
    Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', FDATA.Plano.Reference.Raw,'indice',Results.Reference.Options.Indice};
	GuessPI.WtDyn = MP_muscle_PI_controller(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
	Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', GuessPI.WtDyn,'indice',Results.Reference.Options.Indice,'FixedEndTt', ATt};
	Results.Controller{q} = MP_muscledyn_PI_controller(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
    
    FDATA.Plano.PIController{q}.Person = Person.Controller{q};
    FDATA.Plano.PIController{q}.Options = Options;
    FDATA.Plano.PIController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
    FDATA.Plano.PIController{q}.Raw = Results.Controller{q};
	Graphics(Person.Controller{q},Results.Controller{q},['PI Controller ',int2str(q),'.']);
    
%     Options = {Options{:}, 'OnlyPos', true};
%     Guess = multi_cycle_PI_integrate(Person.Controller{q},Results.Controller{q},true);
%     Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', Guess,'indice',Results.Reference.Options.Indice, 'OnlyPos', true};
%     Results.Controller{q} = multi_cycle_PI_controller(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
%     
%     FDATA.Plano.PSController{q}.Person = Person.Controller{q};
%     FDATA.Plano.PSController{q}.Options = Options;
%     FDATA.Plano.PSController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
%     FDATA.Plano.PSController{q}.Raw = Results.Controller{q};
% 	Graphics(Person.Controller{q},Results.Controller{q},['PS Controller ',int2str(q),'.']);
% FDATA.Plano.Objective(q,:) = [FDATA.Plano.Reference.Results.obj(1), FDATA.Plano.PIController{q}.Results.obj(1), FDATA.Plano.PSController{q}.Results.obj(1)];
    
    FDATA.Plano.Objective(q,:) = [FDATA.Plano.Reference.Results.obj(1), FDATA.Plano.PIController{q}.Results.obj(1)];
    %% Rampa
    
    Person.Controller{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction, 'angle', AngRamp);

	%%
%     Guess = multi_cycle_PI_integrate(Person.Controller{q},FDATA.Rampa.Reference.Raw,false);
	Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', FDATA.Plano.Reference.Raw,'indice',Results.Reference.Options.Indice};
	GuessPI.WtDyn = MP_muscle_PI_controller(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
	Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', GuessPI.WtDyn,'indice',Results.Reference.Options.Indice,'FixedEndTt', ATt};
	Results.Controller{q} = MP_muscledyn_PI_controller(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
    
    FDATA.Rampa.PIController{q}.Person = Person.Controller{q};
    FDATA.Rampa.PIController{q}.Options = Options;
    FDATA.Rampa.PIController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
    FDATA.Rampa.PIController{q}.Raw = Results.Controller{q};
	Graphics(Person.Controller{q},Results.Controller{q},['PI Controller-Rampa',int2str(q),'.']);
%     
%     Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', Contact, 'Guess', Guess,'indice',Results.Reference.Options.Indice, 'OnlyPos', true};
%     Guess = multi_cycle_PI_integrate(Person.Controller{q},Results.Controller{q},true);
%     Results.Controller{q} = multi_cycle_PI_controller(Person.Controller{q}, Num_Cycles, Speed, Distance, Options{:});
%     
%     FDATA.Rampa.PSController{q}.Person = Person.Controller{q};
%     FDATA.Rampa.PSController{q}.Options = Options;
%     FDATA.Rampa.PSController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
%     FDATA.Rampa.PSController{q}.Raw = Results.Controller{q};
% 	Graphics(Person.Controller{q},Results.Controller{q},['PS Controller-Rampa',int2str(q),'.']);
%     
%     FDATA.Rampa.Objective(q,:) = [FDATA.Rampa.Reference.Results.obj(1), FDATA.Rampa.PIController{q}.Results.obj(1), FDATA.Rampa.PSController{q}.Results.obj(1)];
    
end

save([cd,'\Results\','EnergeticRace_','I',int2str(Indice),'V',num2str(Speed),'D',num2str(Distance),'-NEW.mat'],'FDATA','ReferenceModel','Options','Simulations')