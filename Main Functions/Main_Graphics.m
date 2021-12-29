%%

group = 'byAngle';
type = 'SteadyState';
indice = 2;

load([group,'_Indice',int2str(indice),'_',type,'.mat']);

plot_graphics(Person.Reference,Results.Reference,'Reference',[cd,'\Results\',type,'\',group,'\Indice_',int2str(indice)]);

for q=1:size(Simulations,1)
    plot_graphics(Person.Controller{q},Results.Controller{q},['Controller_',int2str(q)],[cd,'\Results\',type,'\',group,'\Indice_',int2str(indice)]);
end

% fileID = fopen(['Objective_',group,'.txt'],'w');
% fprintf(fileID,'----- Objective Values -----\r\n');
% fprintf(fileID,'----------------------------\r\n');
% fprintf(fileID,'- Steady State\t Total \tPhase 1\tPhase 2\r\n');
% fprintf(fileID,'+ Reference   \t');
% fprintf(fileID,'%7.2f\t',Objective.SteadyState.Reference);
% fprintf(fileID,'\r\n');
% fprintf(fileID,'+ Controller\t');
% fprintf(fileID,'%7.2f\t',Objective.SteadyState.Control);

type = 'EnergeticRace';

load([group,'_Indice',int2str(indice),'_',type,'.mat']);

plot_graphics(Person.Reference,Results.Reference,'Reference',[cd,'\Results\',type,'\',group,'\Indice_',int2str(indice)]);

for q=1:size(Simulations,1)
    plot_graphics(Person.Controller{q},Results.Controller{q},['Controller_',int2str(q)],[cd,'\Results\',type,'\',group,'\Indice_',int2str(indice)]);
end



% fprintf(fileID,'\r\n');
% fprintf(fileID,'\r\n');
% fprintf(fileID,'- Energetic Race (indice 2)\r\n');
% fprintf(fileID,'+ Reference \t');
% fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Reference(:,1));
% fprintf(fileID,'\r\n');
% fprintf(fileID,'+ Controller\t');
% fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Control(:,1));
% fprintf(fileID,'\r\n');
% fprintf(fileID,'\r\n');
% fprintf(fileID,'- Energetic Race (indice 3)\r\n');
% fprintf(fileID,'+ Reference \t');
% fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Reference(:,2));
% fprintf(fileID,'\r\n');
% fprintf(fileID,'+ Controller\t');
% fprintf(fileID,'%6.2e\t',Objective.EnergeticRace.Control(:,2));
% fprintf(fileID,'\r\n');
% fprintf(fileID,'----------------------------\r\n');
% fprintf(fileID,'----- Objective Values -----\r\n');
% fprintf(fileID,'----------------------------\r\n');
% fprintf(fileID,'            Type           \t  Mi  \t  Ci  \r\n');
% fprintf(fileID,'- Steady State             \t%6.2f\t%6.2f\r\n',ModelDATA.SteadyState.Mi,ModelDATA.SteadyState.Ci);
% fprintf(fileID,'- Energetic Race (indice 2)\t%6.2f\t%6.2f\r\n',ModelDATA.EnergeticRace.Mi(1),ModelDATA.EnergeticRace.Ci(1));
% fprintf(fileID,'- Energetic Race (indice 3)\t%6.2f\t%6.2f\r\n',ModelDATA.EnergeticRace.Mi(2),ModelDATA.EnergeticRace.Ci(2));
% fprintf(fileID,'----------------------------\r\n');
% fclose(fileID);