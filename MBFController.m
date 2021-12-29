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
Simulations = [0, 0, 0;
               1, 0, 0;
               0, 0, 1;
               1, 0, 1];

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
Graphics(Person.Reference,Results.Reference,'Reference');

for q=1:size(Simulations,1)
    
    %% Model Based Controller
    Person.MBFController{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), 'Fri', ReferenceModel.RRForce*Simulations(q,3), ...
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
	
    FDATA.Plano.MBFController.Person = Person.MBFController{q};
    FDATA.Plano.MBFController.Options = Options;
    FDATA.Plano.MBFController.Results = fvalid(Person.MBFController{q},Results.MBFController{q});
	Graphics(Person.MBFController{q},Results.MBFController{q},['MBF Controller ',int2str(q),'.']);

end

save([cd,'\Results\','MBFControllerN_Indice',int2str(Indice),'_SteadyState','.mat'],'Results','Person','ReferenceModel')
