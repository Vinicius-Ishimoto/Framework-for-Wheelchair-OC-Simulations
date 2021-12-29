%% Reference
% path = 'C:\Users\Vinicius\Google Drive\Tese\Artigos\Artigo da dissertação\LaTeX\figures';
% datapath = 'C:\Users\Vinicius\Google Drive\Tese\Artigos\Artigo da dissertação\LaTeX\datafigures';
pathSS = 'C:\Users\viniciusi.c\Downloads\simu\Results\figures\SteadyState_V0.9';
pathST = 'C:\Users\viniciusi.c\Downloads\simu\Results\figures\Startup_V0.9-1.8';
datapath = 'C:\Users\viniciusi.c\Downloads\simu\Results\datafigures';

st = load('EnergeticRace_I2V0.9D1.8.mat');
ss = load('SteadyState_I2V0.9.mat');
% ss.rt.Reference = fvalid(ss.Person.Reference,ss.Results.Reference);
% st.rt.Reference = fvalid(st.Person.Reference,st.Results.Reference);

%% Steady State (Reference)

Options = {'Density',0.2, 'Movement',false, 'Path',pathSS, 'Name','Stick_Figure_-_SS','Format','eps'};
fgraphicW(ss.FDATA.Plano.Reference.Person,ss.FDATA.Plano.Reference.Results,Options{:});

% Data Analysis
DATA = ss.FDATA.Plano.Reference.Results;

% Least Square
fprintf(['Steady State & ']);
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*ss.FDATA.Plano.Reference.Person.Rr;
u  = interp1(DATA.time,DATA.taup/ss.FDATA.Plano.Reference.Person.R,t,'linear','extrap');
reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);
[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
Y = Y+mean(dx-Y);
fit = goodnessOfFit(Y,dx,'NRMSE');
fitOld = -1;
while (abs((fitOld-fit)/fit) > 0.1)
    [~,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,Y(1));
    Y = Y+mean(dx-Y);
    fitOld = fit;
    fit = goodnessOfFit(Y,dx,'NRMSE');
end
fprintf('%7.2f & ',ReferenceModel.Mass);
fprintf('%7.2f & ',ReferenceModel.Friction);
fprintf('%7.2f\\\\\n',fit*100);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
set(gca, 'FontName', 'Times New Roman')
hold on
	plot(DATA.time,DATA.dtheta*ss.FDATA.Plano.Reference.Person.Rr,'-r','linewidth',2)
	plot(T,Y+mean(dx-Y),'-.b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('time [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Dissertaç£¯\Simulaçµ¥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
print(fig,[pathSS,'\','FIT_-_SS'],'-dpdf','-r0')
fileID = fopen([datapath,'\','FIT_-_SS.txt'],'w');
fprintf(fileID,'%6s %12s %6s %12s\r\n','tlinr','vlinr','treal','vreal');
for i=1:numel(T)
    if i>numel(DATA.time)
        fprintf(fileID,'%6.2f %12.8f %6s %12s\r\n', T(i), Y(i)+mean(dx-Y), '', '');
    else
        fprintf(fileID,'%6.2f %12.8f %6.2f %12.8f\r\n', T(i), Y(i)+mean(dx-Y),DATA.time(i), DATA.dtheta(i)*ss.FDATA.Plano.Reference.Person.Rr);
    end
end
fclose(fileID);

%% Start-up (Reference)

Options = {'Density',0.1, 'Movement',false, 'Path',pathST, 'Name','Stick_Figure_-_ST','Format','eps'};
fgraphicW(st.FDATA.Plano.Reference.Person,st.FDATA.Plano.Reference.Results,Options{:});

% Data Analysis
DATA = st.FDATA.Plano.Reference.Results;

% Least Square
fprintf(['Start-up & ']);
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*st.FDATA.Plano.Reference.Person.Rr;
u  = interp1(DATA.time,DATA.taup/st.FDATA.Plano.Reference.Person.R,t,'linear','extrap');
reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);
[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fit = goodnessOfFit(Y,dx,'NRMSE');
fprintf('%7.2f & ',ReferenceModel.Mass);
fprintf('%7.2f & ',ReferenceModel.Friction);
fprintf('%7.2f\\\\\n',fit*100);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
set(gca, 'FontName', 'Times New Roman')
hold on
	plot(DATA.time,DATA.dtheta*st.FDATA.Plano.Reference.Person.Rr,'-r','linewidth',2)
	plot(T,Y,'-.b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('time [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Dissertaç£¯\Simulaçµ¥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
print(fig,[pathST,'\','FIT_-_ST'],'-dpdf','-r0')
fileID = fopen([datapath,'\','FIT_-_ST.txt'],'w');
fprintf(fileID,'%6s %12s %6s %12s\r\n','tlinr','vlinr','treal','vreal');
for i=1:numel(T)
    if i>numel(DATA.time)
        fprintf(fileID,'%6.2f %12.8f %6s %12s\r\n', T(i), Y(i), '', '');
    else
        fprintf(fileID,'%6.2f %12.8f %6.2f %12.8f\r\n', T(i), Y(i),DATA.time(i), DATA.dtheta(i)*st.FDATA.Plano.Reference.Person.Rr);
    end
end
fclose(fileID);
fprintf('----------------------------------\n');

%% Table (Impedance Variation)

fprintf('----------------------------------\n');
fprintf('- Obj in Torque -- Steady State\n');

fprintf(['Reference & ']);
fprintf('%7.2f & ',ss.FDATA.Plano.Reference.Results.objT(1));
fprintf('-\\\\\n');

for i=1:numel(ss.FDATA.Plano.PIController)
% ss.rt.Controller{i} = fvalid(ss.Person.Controller{i},ss.Results.Controller{i});
fprintf(['$M_{',int2str(ss.Simulations(i,1)*100),'}-C_{',int2str(ss.Simulations(i,2)*100),'}$ & ']);
fprintf('%7.2f & ',ss.FDATA.Plano.PIController{i}.Results.objT(1));
fprintf('%7.2f\\\\\n',ss.FDATA.Plano.PIController{i}.Results.objC(1));
end

fprintf('----------------------------------\n');
fprintf('- Obj in Torque -- Start-up\n');

fprintf(['Reference & ']);
fprintf('%7.2f & ',st.FDATA.Plano.Reference.Results.objT(1));
fprintf('-\\\\\n');

for i=1:numel(st.FDATA.Plano.PIController)
% ss.rt.Controller{i} = fvalid(ss.Person.Controller{i},ss.Results.Controller{i});
fprintf(['$M_{',int2str(ss.Simulations(i,1)*100),'}-C_{',int2str(ss.Simulations(i,2)*100),'}$ & ']);
fprintf('%7.2f & ',st.FDATA.Plano.PIController{i}.Results.objT(1));
fprintf('%7.2f\\\\\n',st.FDATA.Plano.PIController{i}.Results.objC(1));
end

% fprintf(['Reference & ']);
% fprintf('%7.2f & ',st.FDATA.Plano.Reference.Raw.solution.objective);
% fprintf('-\\\\\n');
% 
% for i=1:numel(st.FDATA.Plano.PSController)
% % ss.rt.Controller{i} = fvalid(ss.Person.Controller{i},ss.Results.Controller{i});
% fprintf(['Case ',int2str(i),' & ']);
% fprintf('%7.2f & ',st.FDATA.Plano.PSController{i}.Raw.solution.objective);
% fprintf('%7.2f\\\\\n',st.FDATA.Plano.PSController{i}.Results.objC(1));
% end

fprintf('----------------------------------\n');
fprintf('- Obj in Activation -- Steady State\n');

fprintf(['Reference & ']);
fprintf('%7.2e & ',ss.FDATA.Plano.Reference.Results.obj(1));
fprintf('-\\\\\n');

for i=1:numel(ss.FDATA.Plano.PIController)
% ss.rt.Controller{i} = fvalid(ss.Person.Controller{i},ss.Results.Controller{i});
fprintf(['$M_{',int2str(ss.Simulations(i,1)*100),'}-C_{',int2str(ss.Simulations(i,2)*100),'}$ & ']);
fprintf('%7.2e & ',ss.FDATA.Plano.PIController{i}.Results.obj(1));
fprintf('%7.2f\\\\\n',ss.FDATA.Plano.PIController{i}.Results.objC(1));
end

fprintf('----------------------------------\n');
fprintf('- Obj in Activation -- Start-up\n');

fprintf(['Reference & ']);
fprintf('%7.2e & ',st.FDATA.Plano.Reference.Results.obj(1));
fprintf('-\\\\\n');

for i=1:numel(st.FDATA.Plano.PIController)
% ss.rt.Controller{i} = fvalid(ss.Person.Controller{i},ss.Results.Controller{i});
fprintf(['$M_{',int2str(ss.Simulations(i,1)*100),'}-C_{',int2str(ss.Simulations(i,2)*100),'}$ & ']);
fprintf('%7.2e & ',st.FDATA.Plano.PIController{i}.Results.obj(1));
fprintf('%7.2f\\\\\n',st.FDATA.Plano.PIController{i}.Results.objC(1));
end

rt.Person.Reference = ss.FDATA.Plano.Reference.Person;
rt.Person.Controller = ss.FDATA.Plano.PIController{1}.Person;
rt.Results.Reference = ss.FDATA.Plano.Reference.Raw;
rt.Results.Controller = ss.FDATA.Plano.PIController{1}.Raw;
plot_graphics_combined_II(ss.FDATA.Plano,'Impedance_Variation_-_SS',pathSS,datapath,ss.Simulations,'eps')
plot_graphics_combined_II(st.FDATA.Plano,'Impedance_Variation_-_ST',pathST,datapath,st.Simulations,'eps')
plot_graphics_combined_II(ss.FDATA.Rampa,'Impedance_Var_Ramp_-_SS',pathSS,datapath,ss.Simulations,'eps')
plot_graphics_combined_II(st.FDATA.Rampa,'Impedance_Var_Ramp_-_ST',pathST,datapath,st.Simulations,'eps')

%% Objective Graphic (Angle)

group = 'byAngle';
types = {'SteadyState','EnergeticRace'};
pathSS = 'C:\Users\Vinicius\Google Drive\Tese\Artigos\Artigo da dissertação\LaTeX\figures';
indice = 2;
SS = load([group,'_Indice',int2str(indice),'_',types{1},'.mat']);
ER = load([group,'_Indice',int2str(indice),'_',types{2},'.mat']);

Clr = [0, 0, 0;
    1, 0, 0;
    1, 0, 1;
    0, 0, 1;
    0, 1, 1;
    0, 1, 0];
iClr = 1;
objL = [0,0,0];
Lgd = {'Reference'};

fprintf(['Reference & ']);
fprintf('%7.2f & ',SS.Results.Reference.solution.objective);
fprintf('- & ');
fprintf('%7.2f\\\\\n',SS.Results.Reference.solution.objective);


fig = figure('Name','Velocities and Forces','DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
subplot(2,1,1)
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Controller)
    for q=1:numel(SS.Results.Controller)
        fprintf(['Case ',int2str(q),' & ']);
        DATA = fvalid(SS.Person.Controller{q}, SS.Results.Controller{q});
        objL(q,1) = DATA.obj(1);
        objL(q,2) = DATA.objC(1);
        objL(q,3) = SS.Results.References{q}.solution.objective;
        fprintf('%7.2f & ',objL(q,1));
        fprintf('%7.2f & ',objL(q,2));
        fprintf('%7.2f\\\\\n',objL(q,3));
    end
    plot(SS.Simulations,objL(:,1),'Color',Clr(1,:),'LineStyle','-','linewidth',2)
    plot(SS.Simulations,objL(:,2),'Color',Clr(2,:),'LineStyle','--','linewidth',2)
    plot(SS.Simulations,objL(:,3),'Color',Clr(4,:),'LineStyle','-.','linewidth',2)
hold off
ylabel('Objective [N^2m^2s]'),title('Steady State')
legend('Person','Motor','Person Only','Location','Best')
datap = [SS.Simulations';objL'];
grid

fprintf(['Reference & ']);
fprintf('%7.2f & ',ER.Results.Reference.solution.objective);
fprintf('- & ');
fprintf('%7.2f\\\\\n',ER.Results.Reference.solution.objective);

subplot(2,1,2)
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Controller)
    for q=1:numel(ER.Results.Controller)
        fprintf(['Case ',int2str(q),' & ']);
        DATA = fvalid(ER.Person.Controller{q}, ER.Results.Controller{q});
        objL(q,1) = DATA.obj(1);
        objL(q,2) = DATA.objC(1);
        objL(q,3) = ER.Results.References{q}.solution.objective;
        fprintf('%7.2f & ',objL(q,1));
        fprintf('%7.2f & ',objL(q,2));
        fprintf('%7.2f\\\\\n',objL(q,3));
    end
    plot(ER.Simulations,objL(:,1),'Color',Clr(1,:),'LineStyle','-','linewidth',2)
    plot(ER.Simulations,objL(:,2),'Color',Clr(2,:),'LineStyle','--','linewidth',2)
    plot(ER.Simulations,objL(:,3),'Color',Clr(4,:),'LineStyle','-.','linewidth',2)
hold off
xlabel('Angle [°]'),ylabel('Objective [N^2m^2s]'),title('Start-up')
legend('Person','Motor','Person Only','Location','Best')
grid
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(fig,[pathSS,'\Objective_-_',group],'-dpdf','-r0')
datap = [datap;objL'];
fileID = fopen([datapath,'\','Objective_-_',group,'.txt'],'w');
fprintf(fileID,'%6s %12s %12s %12s %12s %12s %12s\r\n','cases','ctrp1','ctrm1','rfrc1','ctrp2','ctrm2','rfrc2');
fprintf(fileID,'%6.2f %12.6f %12.6f %12.6f %12.6f %12.6f %12.6f\r\n',datap);
fclose(fileID);

%% Objective Graphic (Pavement)

group = 'byRollingResistance';
types = {'SteadyState','EnergeticRace'};
pathSS = 'C:\Users\Vinicius\Google Drive\Tese\Artigos\Artigo da dissertação\LaTeX\figures';
indice = 2;
SS = load([group,'_Indice',int2str(indice),'_',types{1},'.mat']);
ER = load([group,'_Indice',int2str(indice),'_',types{2},'.mat']);

Clr = [0, 0, 0;
    1, 0, 0;
    1, 0, 1;
    0, 0, 1;
    0, 1, 1;
    0, 1, 0];
iClr = 1;
objL = [0,0,0];
Lgd = {'Reference'};

fprintf(['Reference & ']);
fprintf('%7.2f & ',SS.Results.Reference.solution.objective);
fprintf('- & ');
fprintf('%7.2f\\\\\n',SS.Results.Reference.solution.objective);

fig = figure('Name','Velocities and Forces','DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
subplot(2,1,1)
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Controller)
    for q=1:numel(SS.Results.Controller)
        fprintf(['Case ',int2str(q),' & ']);
        DATA = fvalid(SS.Person.Controller{q}, SS.Results.Controller{q});
        objL(q,1) = DATA.obj(1);
        objL(q,2) = DATA.objC(1);
        objL(q,3) = SS.Results.References{q}.solution.objective;
        fprintf('%7.2f & ',objL(q,1));
        fprintf('%7.2f & ',objL(q,2));
        fprintf('%7.2f\\\\\n',objL(q,3));
    end
    plot(SS.Simulations,objL(:,1),'Color',Clr(1,:),'LineStyle','-','linewidth',2)
    plot(SS.Simulations,objL(:,2),'Color',Clr(2,:),'LineStyle','--','linewidth',2)
    plot(SS.Simulations,objL(:,3),'Color',Clr(4,:),'LineStyle','-.','linewidth',2)
hold off
ylabel('Objective [N^2m^2s]'),title('Steady State')
legend('Person','Motor','Person Only','Location','Best')
datap = [SS.Simulations';objL'];
grid

fprintf(['Reference & ']);
fprintf('%7.2f & ',ER.Results.Reference.solution.objective);
fprintf('- & ');
fprintf('%7.2f\\\\\n',ER.Results.Reference.solution.objective);

subplot(2,1,2)
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Controller)
    for q=1:numel(ER.Results.Controller)
        fprintf(['Case ',int2str(q),' & ']);
        DATA = fvalid(ER.Person.Controller{q}, ER.Results.Controller{q});
        objL(q,1) = DATA.obj(1);
        objL(q,2) = DATA.objC(1);
        objL(q,3) = ER.Results.References{q}.solution.objective;
        fprintf('%7.2f & ',objL(q,1));
        fprintf('%7.2f & ',objL(q,2));
        fprintf('%7.2f\\\\\n',objL(q,3));
    end
    plot(ER.Simulations,objL(:,1),'Color',Clr(1,:),'LineStyle','-','linewidth',2)
    plot(ER.Simulations,objL(:,2),'Color',Clr(2,:),'LineStyle','--','linewidth',2)
    plot(ER.Simulations,objL(:,3),'Color',Clr(4,:),'LineStyle','-.','linewidth',2)
hold off
xlabel('Rolling Resistance [N]'),ylabel('Objective [N^2m^2s]'),title('Start-up')
legend('Person','Motor','Person Only','Location','Best')
grid
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(fig,[pathSS,'\Objective_-_',group],'-dpdf','-r0')
datap = [datap;objL'];
fileID = fopen([datapath,'\','Objective_-_',group,'.txt'],'w');
fprintf(fileID,'%6s %12s %12s %12s %12s %12s %12s\r\n','cases','ctrp1','ctrm1','rfrc1','ctrp2','ctrm2','rfrc2');
fprintf(fileID,'%6.2f %12.6f %12.6f %12.6f %12.6f %12.6f %12.6f\r\n',datap);
fclose(fileID);