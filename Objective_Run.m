%% Objective

%% Steady State
load([cd,'\Results\Steady State\Reference_SteadyState.mat']);
DATA = fvalid(Person.PAR,Results);
Objective.SteadyState.Reference = DATA.obj;
load([cd,'\Results\Steady State\Control_SteadyState.mat']);
DATA = fvalid(Person.PAR,Results);
Objective.SteadyState.Control = DATA.obj;
ModelDATA.SteadyState.Mi = Results.Options.ImpedanceMass;
ModelDATA.SteadyState.Ci = Results.Options.ImpedanceFriction;
Objective.SteadyState.Total = [Objective.SteadyState.Reference, Objective.SteadyState.Control];

%% Energetic Race
load([cd,'\Results\Energetic Race\Reference\indice_2\Reference_EnergeticRace.mat']);
DATA = fvalid(Person.PAR,Results);
Objective.EnergeticRace.Reference = DATA.obj;
load([cd,'\Results\Energetic Race\Controller\indice_2\Control_EnergeticRace.mat']);
DATA = fvalid(Person.PAR,Results);
Objective.EnergeticRace.Control = DATA.obj;
ModelDATA.EnergeticRace.Mi = Results.Options.ImpedanceMass;
ModelDATA.EnergeticRace.Ci = Results.Options.ImpedanceFriction;
load([cd,'\Results\Energetic Race\Reference\indice_3\Reference_EnergeticRace.mat']);
DATA = fvalid(Person.PAR,Results);
Objective.EnergeticRace.Reference = [Objective.EnergeticRace.Reference, DATA.obj];
load([cd,'\Results\Energetic Race\Controller\indice_3\Control_EnergeticRace.mat']);
DATA = fvalid(Person.PAR,Results);
Objective.EnergeticRace.Control = [Objective.EnergeticRace.Control, DATA.obj];
ModelDATA.EnergeticRace.Mi = [ModelDATA.EnergeticRace.Mi, Results.Options.ImpedanceMass];
ModelDATA.EnergeticRace.Ci = [ModelDATA.EnergeticRace.Ci, Results.Options.ImpedanceFriction];
Objective.EnergeticRace.Total = [Objective.EnergeticRace.Reference, Objective.EnergeticRace.Control];

fileID = fopen('Objective_Values.txt','w');
fprintf(fileID,'----- Objective Values -----\r\n');
fprintf(fileID,'----------------------------\r\n');
fprintf(fileID,'- Steady State\t Total \tPhase 1\tPhase 2\r\n');
fprintf(fileID,'+ Reference   \t');
fprintf(fileID,'%7.2f\t',Objective.SteadyState.Reference);
fprintf(fileID,'\r\n');
fprintf(fileID,'+ Controller\t');
fprintf(fileID,'%7.2f\t',Objective.SteadyState.Control);
fprintf(fileID,'\r\n');
fprintf(fileID,'\r\n');
fprintf(fileID,'- Energetic Race (indice 2)\r\n');
fprintf(fileID,'+ Reference \t');
fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Reference(:,1));
fprintf(fileID,'\r\n');
fprintf(fileID,'+ Controller\t');
fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Control(:,1));
fprintf(fileID,'\r\n');
fprintf(fileID,'\r\n');
fprintf(fileID,'- Energetic Race (indice 3)\r\n');
fprintf(fileID,'+ Reference \t');
fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Reference(:,2));
fprintf(fileID,'\r\n');
fprintf(fileID,'+ Controller\t');
fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Control(:,2));
fprintf(fileID,'\r\n');
fprintf(fileID,'----------------------------\r\n');
fprintf(fileID,'----- Objective Values -----\r\n');
fprintf(fileID,'----------------------------\r\n');
fprintf(fileID,'            Type           \t  Mi  \t  Ci  \r\n');
fprintf(fileID,'- Steady State             \t%6.2f\t%6.2f\r\n',ModelDATA.SteadyState.Mi,ModelDATA.SteadyState.Ci);
fprintf(fileID,'- Energetic Race (indice 2)\t%6.2f\t%6.2f\r\n',ModelDATA.EnergeticRace.Mi(1),ModelDATA.EnergeticRace.Ci(1));
fprintf(fileID,'- Energetic Race (indice 3)\t%6.2f\t%6.2f\r\n',ModelDATA.EnergeticRace.Mi(2),ModelDATA.EnergeticRace.Ci(2));
fprintf(fileID,'----------------------------\r\n');
fclose(fileID);