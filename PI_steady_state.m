% Simulações Com controle variado

addpath (genpath ('C:\Users\viniciusi.c\Downloads\simu'));
addpath(genpath ('Z:\Controle �timo'));

%% Person data
Person.Height = 1.7;
Person.Mass = 70;
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass);
Max_Shoulder = 124; % Maximum torque in shoulder
Max_Elbow = 90; % Maximum torque in elbow
AngRamp = 3;
Indice = 2;
ATt = true;
% Mass, Friction
Simulations = [1, 1;
               0.5, 1;
               1, 0.5;
			   0.5, 0.5];

%% Simulation data (1) Steady-State
Speed = 0.9;
Flag = 2; % 1 == fixed contact angle; 2 == free contact angle
Contact = 50;
Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice};
Guess.NoDyn = steady_state_wheelchair(Person.Reference, Speed, Options{:});

Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice, 'Guess', Guess.NoDyn};
Guess.WtDyn = SS_muscle_wheelchair(Person.Reference, Speed, Options{:});

Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice, 'Guess', Guess.WtDyn, 'FixedEndTt', ATt};
Results.Reference = SS_muscledyn_wheelchair(Person.Reference, Speed, Options{:});

%% Data Analysis
st = load('EnergeticRace_I2V0.9D1.6.mat');
DATA = fvalid(Person.Reference, Results.Reference);

% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*Person.Reference.Rr;
u  = interp1(DATA.time,DATA.taup/Person.Reference.R,t,'linear','extrap');

% reg = [dx(1:end-1),u(1:end-1)];
% thetaf = pinv(reg)*dx(2:end);

ReferenceModel = st.ReferenceModel;

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*Person.Reference.Rr,'-r','linewidth',2)
	plot(T,Y+mean(dx-Y),'-b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

FDATA.Plano.Reference.Person = Person.Reference;
FDATA.Plano.Reference.Options = Options;
FDATA.Plano.Reference.Results = fvalid(Person.Reference,Results.Reference);
FDATA.Plano.Reference.Raw = Results.Reference;
FDATA.ReferenceModel = ReferenceModel;
Graphics(Person.Reference,Results.Reference,'Reference - Plano');

% Guesses1 = load('byAssistiveController_Indice2_SteadyState.mat');

%%
Person.Reference = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, 'angle', AngRamp);
Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice};
Guess.NoDyn = steady_state_wheelchair(Person.Reference, Speed, Options{:});

Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice, 'Guess', Guess.NoDyn};
Guess.WtDyn = SS_muscle_wheelchair(Person.Reference, Speed, Options{:});

Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow, 'indice', Indice, 'Guess', Guess.WtDyn, 'FixedEndTt', ATt};
Results.Reference = SS_muscledyn_wheelchair(Person.Reference, Speed, Options{:});

FDATA.Rampa.Reference.Person = Person.Reference;
FDATA.Rampa.Reference.Options = Options;
FDATA.Rampa.Reference.Results = fvalid(Person.Reference,Results.Reference);
FDATA.Rampa.Reference.Raw = Results.Reference;

Graphics(Person.Reference,Results.Reference,'Reference - Rampa');

% Guesses2 = load('byAssistiveController_Indice2_SteadyState.mat');

for q=1:size(Simulations,1)

	%% Simulation data
    Person.Controller{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction);
% 	Person.Controller{q}.Mi = ReferenceModel.Mass*Simulations(q,1);
%     Person.Controller{q}.Ci = ReferenceModel.Friction*Simulations(q,2);
    Speed = Results.Reference.Options.MeanVelocity;
	Max_Shoulder = Results.Reference.Options.MaxShoulder; % Maximum torque in shoulder
	Max_Elbow = Results.Reference.Options.MaxElbow; % Maximum torque in elbow
% 	Guess = multi_cycle_PID_integrate(Person.Controller{q}, Results.Reference);
    
	Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',FDATA.Plano.Reference.Raw, 'indice', Indice, 'FixedEndTt', ATt};
	Results.Controller{q} = SS_muscledyn_PI_controller(Person.Controller{q}, Speed, Options{:});
    
    FDATA.Plano.PIController{q}.Person = Person.Controller{q};
    FDATA.Plano.PIController{q}.Options = Options;
    FDATA.Plano.PIController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
    FDATA.Plano.PIController{q}.Raw = Results.Controller{q};
	Graphics(Person.Controller{q},Results.Controller{q},['PI Controller ',int2str(q),'.']);
    
%     Options = {Options{:}, 'OnlyPos', true};
%     Results.Controller{q} = steady_state_PI_controller(Person.Controller{q}, Speed, Options{:});
%     
%     FDATA.Plano.PSController{q}.Person = Person.Controller{q};
%     FDATA.Plano.PSController{q}.Options = Options;
%     FDATA.Plano.PSController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
%     FDATA.Plano.PSController{q}.Raw = Results.Controller{q};
% 	Graphics(Person.Controller{q},Results.Controller{q},['PS Controller ',int2str(q),'.']);
%     FDATA.Plano.Objective(q,:) = [FDATA.Plano.Reference.Results.obj(1), FDATA.Plano.PIController{q}.Results.obj(1), FDATA.Plano.PSController{q}.Results.obj(1)];
    
    FDATA.Plano.Objective(q,:) = [FDATA.Plano.Reference.Results.obj(1), FDATA.Plano.PIController{q}.Results.obj(1)];
    
    %% Angle
    Person.Controller{q} = wheelchair_parameters('height',Person.Height, 'mass',Person.Mass, ...
        'Mi', ReferenceModel.Mass*Simulations(q,1), 'Ci', ReferenceModel.Friction*Simulations(q,2), ...
        'Mr', ReferenceModel.Mass, 'Cr', ReferenceModel.Friction, 'angle', AngRamp);
    
    Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',FDATA.Rampa.Reference.Raw, 'indice', Indice, 'FixedEndTt', ATt};
	Results.Controller{q} = SS_muscledyn_PI_controller(Person.Controller{q}, Speed, Options{:});
    
    FDATA.Rampa.PIController{q}.Person = Person.Controller{q};
    FDATA.Rampa.PIController{q}.Options = Options;
    FDATA.Rampa.PIController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
    FDATA.Rampa.PIController{q}.Raw = Results.Controller{q};
	Graphics(Person.Controller{q},Results.Controller{q},['PI Controller - Rampa',int2str(q),'.']);
    
%     Options = {Options{:}, 'OnlyPos', true};
%     Results.Controller{q} = steady_state_PI_controller(Person.Controller{q}, Speed, Options{:});
%     
%     FDATA.Rampa.PSController{q}.Person = Person.Controller{q};
%     FDATA.Rampa.PSController{q}.Options = Options;
%     FDATA.Rampa.PSController{q}.Results = fvalid(Person.Controller{q},Results.Controller{q});
%     FDATA.Rampa.PSController{q}.Raw = Results.Controller{q};
% 	Graphics(Person.Controller{q},Results.Controller{q},['PS Controller - Rampa',int2str(q),'.']);
%     FDATA.Rampa.Objective(q,:) = [FDATA.Rampa.Reference.Results.obj(1), FDATA.Rampa.PIController{q}.Results.obj(1), FDATA.Rampa.PSController{q}.Results.obj(1)];
    
    FDATA.Rampa.Objective(q,:) = [FDATA.Rampa.Reference.Results.obj(1), FDATA.Rampa.PIController{q}.Results.obj(1)];
    

end

save([cd,'\Results\','SteadyState_','I',int2str(Indice),'V',num2str(Speed),'.mat'],'Results','Person','ReferenceModel','Options','Simulations','FDATA')

