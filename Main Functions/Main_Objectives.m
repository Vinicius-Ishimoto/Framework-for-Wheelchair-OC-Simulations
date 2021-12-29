%%

group = 'byAngle';
type = 'SteadyState';
indice = 2;

load([group,'_Indice',int2str(indice),'_',type,'.mat']);

fileID = fopen([cd,'\Results\Objective_',group,'.txt'],'w');
fprintf(fileID,'----- Objective Values -----\r\n');
fprintf(fileID,'----------------------------\r\n');
fprintf(fileID,'--------- Index   2 --------\r\n');
fprintf(fileID,'----------------------------\r\n');
fprintf(fileID,'- Steady State  \t  Total\r\n');
fprintf(fileID,'+ Reference     \t');
fprintf(fileID,'%7.2f',Results.Reference.solution.objective);
fprintf(fileID,'\r\n');

% fprintf(fileID,'%7.2f\t',Objective.SteadyState.Control);

for q=1:size(Simulations,1)
    fprintf(fileID,['+ Controller   ',int2str(q),'\t']);
    fprintf(fileID,'%7.2f\r\n',Results.Controller{q}.solution.objective);
end

fprintf(fileID,'\r\n');
fprintf(fileID,'\r\n');

type = 'EnergeticRace';
load([group,'_Indice',int2str(indice),'_',type,'.mat']);

fprintf(fileID,'- Energetic Race\t  Total\r\n');
fprintf(fileID,'+ Reference     \t');
fprintf(fileID,'%7.2f\t',Results.Reference.solution.objective);
fprintf(fileID,'\r\n');

for q=1:size(Simulations,1)
    fprintf(fileID,['+ Controller   ',int2str(q),'\t']);
    fprintf(fileID,'%7.2f\r\n',Results.Controller{q}.solution.objective);
end

fprintf(fileID,'\r\n');
fprintf(fileID,'\r\n');

%%
type = 'SteadyState';
indice = 3;

load([group,'_Indice',int2str(indice),'_',type,'.mat']);

fprintf(fileID,'----------------------------\r\n');
fprintf(fileID,'--------- Index   3 --------\r\n');
fprintf(fileID,'----------------------------\r\n');
fprintf(fileID,'- Steady State  \t  Total\r\n');
fprintf(fileID,'+ Reference     \t');
fprintf(fileID,'%7.2f',Results.Reference.solution.objective);
fprintf(fileID,'\r\n');

% fprintf(fileID,'%7.2f\t',Objective.SteadyState.Control);

for q=1:size(Simulations,1)
    fprintf(fileID,['+ Controller   ',int2str(q),'\t']);
    fprintf(fileID,'%7.2f\r\n',Results.Controller{q}.solution.objective);
end

fprintf(fileID,'\r\n');
fprintf(fileID,'\r\n');

type = 'EnergeticRace';
load([group,'_Indice',int2str(indice),'_',type,'.mat']);

fprintf(fileID,'- Energetic Race\t  Total\r\n');
fprintf(fileID,'+ Reference     \t');
fprintf(fileID,'%7.2f\t',Results.Reference.solution.objective);
fprintf(fileID,'\r\n');

for q=1:size(Simulations,1)
    fprintf(fileID,['+ Controller   ',int2str(q),'\t']);
    fprintf(fileID,'%7.2f\r\n',Results.Controller{q}.solution.objective);
end

fclose(fileID);