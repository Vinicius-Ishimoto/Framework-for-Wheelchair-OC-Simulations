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
    Person.PSTController{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), 'Fri', ReferenceModel.RRForce*Simulations(q,3), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);
% 	Person.Controller{q}.Mi = ReferenceModel.Mass*Simulations(q,1);
%     Person.Controller{q}.Ci = ReferenceModel.Friction*Simulations(q,2);
    Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
	Guess = multi_cycle_PID_integrate(Person.PSTController{q}, Results.Reference);
    
	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess, 'indice', Indice};
	Results.PSTController{q} = steady_state_PIDSAT_controller(Person.PSTController{q}, Speed, Options{:});
	
    FDATA.Plano.PSTController{q}.Person = Person.PSTController{q};
    FDATA.Plano.PSTController{q}.Options = Options;
    FDATA.Plano.PSTController{q}.Results = fvalid(Person.PSTController{q},Results.PSTController{q});
    FDATA.Plano.PSTController{q}.Raw = Results.PSTController{q};
	Graphics(Person.PSTController{q},Results.PSTController{q},['PID SAT Controller ',int2str(q),'.']);
    
    FDATA.Plano.Objective(q,:) = [FDATA.Plano.Reference.Results.obj(1), FDATA.Plano.PSTController{q}.Results.obj(1)];

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
    Person.PSTController{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle',Angle, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), 'Fri', ReferenceModel.RRForce*Simulations(q,3), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);

    Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
	Guess = multi_cycle_PID_integrate(Person.PSTController{q}, Results.Reference);
    
	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess, 'indice', Indice};
	Results.PSTController{q} = steady_state_PIDSAT_controller(Person.PSTController{q}, Speed, Options{:});
	
    FDATA.Rampa.PSTController{q}.Person = Person.PSTController{q};
    FDATA.Rampa.PSTController{q}.Options = Options;
    FDATA.Rampa.PSTController{q}.Results = fvalid(Person.PSTController{q},Results.PSTController{q});
    FDATA.Rampa.PSTController{q}.Raw = Results.PSTController{q};
	Graphics(Person.PSTController{q},Results.PSTController{q},['PID SAT Controller ',int2str(q),'.']);
    
    FDATA.Rampa.Objective(q,:) = [FDATA.Rampa.Reference.Results.obj(1), FDATA.Rampa.PSTController{q}.Results.obj(1)];

end

save([cd,'\Results\','PIDSATController1_Indice',int2str(Indice),'_SteadyState','.mat'],'Results','Person','ReferenceModel','FDATA')
