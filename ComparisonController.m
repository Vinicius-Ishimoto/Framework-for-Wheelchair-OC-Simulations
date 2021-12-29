% Simulações Com controle variado

addpath (genpath ('C:\Users\viniciusi.c\Downloads\simu'));
addpath(genpath ('Z:\Controle �timo'));

%% Person data
Person.Height = 1.7;
Person.Mass = 70;
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass);
Max_Shoulder = 124; % Maximum torque in shoulder
Max_Elbow = 90; % Maximum torque in elbow
Indice = 2;
% Mass, Friction
Simulations = [0.5, 0, 1;
               1, 0, 0.5;
               0.5, 0, 0.5];

%% Simulation data (1) Steady-State
Speed = 0.9;
Flag = 2; % 1 == fixed contact angle; 2 == free contact angle
Contact = 50;
Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice};
Results.Reference = steady_state_wheelchair(Person.Reference, Speed, Options{:});

ReferenceModel.Mass = Person.Reference.mCd+Person.Mass+Person.Reference.mr;
ReferenceModel.Friction = Person.Reference.dFric;
ReferenceModel.RRForce = Person.Reference.Frr;

FDATA.Plano.Reference.Person = Person.Reference;
FDATA.Plano.Reference.Options = Options;
FDATA.Plano.Reference.Results = fvalid(Person.Reference,Results.Reference);
FDATA.Plano.Reference.Raw = Results.Reference;
Graphics(Person.Reference,Results.Reference,'Reference');

for q=1:size(Simulations,1)

	%% PID Controller
    Person.PIDController{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), 'Fri', ReferenceModel.RRForce*Simulations(q,3), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);
% 	Person.Controller{q}.Mi = ReferenceModel.Mass*Simulations(q,1);
%     Person.Controller{q}.Ci = ReferenceModel.Friction*Simulations(q,2);
    Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
	Guess = multi_cycle_PID_integrate(Person.PIDController{q}, Results.Reference);
    
	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess, 'indice', Indice};
	Results.PIDController{q} = steady_state_PID_controller(Person.PIDController{q}, Speed, Options{:});
	
    FDATA.Plano.PIDController{q}.Person = Person.PIDController{q};
    FDATA.Plano.PIDController{q}.Options = Options;
    FDATA.Plano.PIDController{q}.Results = fvalid(Person.PIDController{q},Results.PIDController{q});
    FDATA.Plano.PIDController{q}.Raw = Results.PIDController{q};
	Graphics(Person.PIDController{q},Results.PIDController{q},['PID Controller ',int2str(q),'.']);
    
    %% Model Based Controller
%     Mir = ReferenceModel.Mass + Person.Reference.Jm/Person.Reference.Rr^2 - ReferenceModel.Mass*Simulations(q,1);
%     Cir = Person.Reference.Bfric/Person.Reference.Rr^2 - ReferenceModel.Friction*Simulations(q,2);
    Mir = ReferenceModel.Mass - ReferenceModel.Mass*Simulations(q,1);
    Cir = ReferenceModel.Friction - ReferenceModel.Friction*Simulations(q,2);
    Fir = ReferenceModel.RRForce - ReferenceModel.RRForce*Simulations(q,3);
    Person.MBFController{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', Mir, 'Ci', Cir, 'Fri', Fir, ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);
% 	Person.Controller{q}.Mi = ReferenceModel.Mass*Simulations(q,1);
%     Person.Controller{q}.Ci = ReferenceModel.Friction*Simulations(q,2);
    Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
% 	Guess = multi_cycle_PID_integrate(Person.MBFController{q}, Results.Reference);
    Guess = Results.Reference;

	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess, 'indice', Indice};
	Results.MBFController{q} = steady_state_MBF_controller(Person.MBFController{q}, Speed, Options{:});
	
    FDATA.Plano.MBFController{q}.Person = Person.MBFController{q};
    FDATA.Plano.MBFController{q}.Options = Options;
    FDATA.Plano.MBFController{q}.Results = fvalid(Person.MBFController{q},Results.MBFController{q});
    FDATA.Plano.MBFController{q}.Raw = Results.MBFController{q};
	Graphics(Person.MBFController{q},Results.MBFController{q},['MBF Controller ',int2str(q),'.']);
    
    FDATA.Plano.Objective(q,:) = [FDATA.Plano.Reference.Results.obj(1), FDATA.Plano.PIDController{q}.Results.obj(1), FDATA.Plano.MBFController{q}.Results.obj(1)];

end

%% Simulation data (2) Angle

Angle = atand(1/12);
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle',Angle);
Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice};
Results.Reference = steady_state_wheelchair(Person.Reference, Speed, Options{:});


% ReferenceModel.Mass = Person.Reference.Jmt/Person.Reference.Rr^2;
% ReferenceModel.Friction = Person.Reference.dFric;
% ReferenceModel.RRForce = Person.Reference.Frr;


FDATA.Rampa.Reference.Person = Person.Reference;
FDATA.Rampa.Reference.Options = Options;
FDATA.Rampa.Reference.Results = fvalid(Person.Reference,Results.Reference);
FDATA.Rampa.Reference.Raw = Results.Reference;
Graphics(Person.Reference,Results.Reference,'Reference');

for q=1:size(Simulations,1)

	%% PID Controller
    Person.PIDController{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle',Angle, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), 'Fri', ReferenceModel.RRForce*Simulations(q,3), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);

    Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
	Guess = multi_cycle_PID_integrate(Person.PIDController{q}, Results.Reference);
    
	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess, 'indice', Indice};
	Results.PIDController{q} = steady_state_PID_controller(Person.PIDController{q}, Speed, Options{:});
	
    FDATA.Rampa.PIDController{q}.Person = Person.PIDController{q};
    FDATA.Rampa.PIDController{q}.Options = Options;
    FDATA.Rampa.PIDController{q}.Results = fvalid(Person.PIDController{q},Results.PIDController{q});
    FDATA.Rampa.PIDController{q}.Raw = Results.PIDController{q};
	Graphics(Person.PIDController{q},Results.PIDController{q},['PID Controller ',int2str(q),'.']);
    
    %% Model Based Controller
%     Mir = ReferenceModel.Mass + Person.Reference.Jm/Person.Reference.Rr^2 - ReferenceModel.Mass*Simulations(q,1);
%     Cir = Person.Reference.Bfric/Person.Reference.Rr^2 - ReferenceModel.Friction*Simulations(q,2);
    Mir = ReferenceModel.Mass - ReferenceModel.Mass*Simulations(q,1);
    Cir = ReferenceModel.Friction - ReferenceModel.Friction*Simulations(q,2);
    Fir = ReferenceModel.RRForce - ReferenceModel.RRForce*Simulations(q,3);
    Person.MBFController{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle',Angle, ...
        'Mi', Mir, 'Ci', Cir, 'Fri', Fir, ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);

    Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
    Guess = Results.Reference;

	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess, 'indice', Indice};
	Results.MBFController{q} = steady_state_MBF_controller(Person.MBFController{q}, Speed, Options{:});
	
    FDATA.Rampa.MBFController{q}.Person = Person.MBFController{q};
    FDATA.Rampa.MBFController{q}.Options = Options;
    FDATA.Rampa.MBFController{q}.Results = fvalid(Person.MBFController{q},Results.MBFController{q});
    FDATA.Rampa.MBFController{q}.Raw = Results.MBFController{q};
	Graphics(Person.MBFController{q},Results.MBFController{q},['MBF Controller ',int2str(q),'.']);
    
    FDATA.Rampa.Objective(q,:) = [FDATA.Rampa.Reference.Results.obj(1), FDATA.Rampa.PIDController{q}.Results.obj(1), FDATA.Rampa.MBFController{q}.Results.obj(1)];

end

save([cd,'\Results\','ComparisonController6_Indice',int2str(Indice),'_SteadyState','.mat'],'Results','Person','ReferenceModel','FDATA')
