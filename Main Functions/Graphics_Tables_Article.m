

%%

path = 'C:\Users\Vinicius\Google Drive\Tese\Artigos\Artigo da dissertação\LaTeX\figures';

group = 'byAngle';
type = 'SteadyState';
indice = 2;

load([group,'_Indice',int2str(indice),'_',type,'.mat']);

plot_graphics_combined(Person,Results,['i',int2str(indice),'_',group,'_',type],path);

% Data Analysis
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

type = 'EnergeticRace';

load([group,'_Indice',int2str(indice),'_',type,'.mat']);

% Data Analysis
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
	plot(T,Y,'-b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

plot_graphics_combined(Person,Results,['i',int2str(indice),'_',group,'_',type],path);

%% Objective

types = {'SteadyState','EnergeticRace'};

SS = load([group,'_Indice',int2str(indice),'_',types{1},'.mat']);
ER = load([group,'_Indice',int2str(indice),'_',types{2},'.mat']);

fprintf(['Reference & ']);
fprintf('%7.2f & ',SS.Results.Reference.solution.objective);
fprintf('%7.2f\\\\\n',ER.Results.Reference.solution.objective);

for q=1:size(Simulations,1)
    fprintf(['Case ',num2str(q),' & ']);
    fprintf('%7.2f & ',SS.Results.Controller{q}.solution.objective);
    fprintf('%7.2f\\\\\n',ER.Results.Controller{q}.solution.objective);
end

%% Mass/Friction

types = {'SteadyState','EnergeticRace'};

% SS = load([group,'_Indice',int2str(indice),'_',types{1},'.mat']);
% ER = load([group,'_Indice',int2str(indice),'_',types{2},'.mat']);

fprintf(['Reference & ']);
fprintf('- & ');
fprintf('- & ');
fprintf('- & ');
fprintf('-\\\\\n');

for q=1:size(Simulations,1)
    fprintf(['Case ',num2str(q),' & ']);
    fprintf('%7.2f & ',SS.Results.Controller{q}.Options.ImpedanceMass);
    fprintf('%7.2f & ',SS.Results.Controller{q}.Options.ImpedanceFriction);
    fprintf('%7.2f & ',ER.Results.Controller{q}.Options.ImpedanceMass);
    fprintf('%7.2f\\\\\n',ER.Results.Controller{q}.Options.ImpedanceFriction);
end

%% FIT

fprintf(['Reference & ']);

% Data Analysis (SS)
DATA = fvalid(SS.Person.Reference, SS.Results.Reference);
% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*SS.Person.Reference.Rr;
u  = interp1(DATA.time,DATA.taup/SS.Person.Reference.R,t,'linear','extrap');
reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);
IReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
IReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);
[~,Y] = ode45(@(tv,vt) (1/IReferenceModel.Mass)*(interp1(t,u,tv)-IReferenceModel.Friction*vt),t,0);
Y = Y+mean(dx-Y);
fit = goodnessOfFit(Y,dx,'NRMSE');
fitOld = -1;
while (abs((fitOld-fit)/fit) > 0.1)
    [~,Y] = ode45(@(tv,vt) (1/IReferenceModel.Mass)*(interp1(t,u,tv)-IReferenceModel.Friction*vt),t,Y(1));
    Y = Y+mean(dx-Y);
    fitOld = fit;
    fit = goodnessOfFit(Y,dx,'NRMSE');
end
fprintf('%7.2f & ',fit*100);

% Data Analysis (ER)
DATA = fvalid(ER.Person.Reference, ER.Results.Reference);
% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*ER.Person.Reference.Rr;
u  = interp1(DATA.time,DATA.taup/ER.Person.Reference.R,t,'linear','extrap');
reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);
IReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
IReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);
[~,Y] = ode45(@(tv,vt) (1/IReferenceModel.Mass)*(interp1(t,u,tv)-IReferenceModel.Friction*vt),t,0);
% Y = Y+mean(dx-Y);
fit = goodnessOfFit(Y,dx,'NRMSE');
fprintf('%7.2f\\\\\n',fit*100);

for q=1:size(Simulations,1)
    fprintf(['Case ',num2str(q),' & ']);
    DATA = fvalid(SS.Person.Controller{q}, SS.Results.Controller{q});
    fit = goodnessOfFit(DATA.dtheta*SS.Person.Controller{q}.Rr,DATA.dveli,'NRMSE');
    fprintf('%7.2f & ',fit*100);

    DATA = fvalid(ER.Person.Controller{q}, ER.Results.Controller{q});
    fit = goodnessOfFit(DATA.dtheta*ER.Person.Controller{q}.Rr,DATA.dveli,'NRMSE');
    fprintf('%7.2f\\\\\n',fit*100);
end

%% Objective Graphic

group = 'byAngle';
types = {'SteadyState','EnergeticRace'};
path = 'C:\Users\Vinicius\Google Drive\Tese\Artigos\Artigo da dissertação\LaTeX\figures';
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

fig = figure('Name','Velocities and Forces','DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
subplot(2,1,1)
hold on
    % Data Analysis (Controller)
    for q=1:numel(Results.Controller)
        DATA = fvalid(SS.Person.Controller{q}, SS.Results.Controller{q});
        objL(q,1) = DATA.obj(1);
        objL(q,2) = DATA.objC(1);
        objL(q,3) = SS.Results.References{q}.solution.objective;
    end
    plot(SS.Simulations,objL(:,1),'Color',Clr(1,:),'linewidth',2)
    plot(SS.Simulations,objL(:,2),'Color',Clr(2,:),'linewidth',2)
    plot(SS.Simulations,objL(:,3),'Color',Clr(4,:),'linewidth',2)
hold off
xlabel('Angle [°]'),ylabel('Objective [N^2]'),title('Steady State')
legend('Person','Motor','Person Only','Location','Best')
grid
subplot(2,1,2)
hold on
    % Data Analysis (Controller)
    for q=1:numel(Results.Controller)
        DATA = fvalid(ER.Person.Controller{q}, ER.Results.Controller{q});
        objL(q,1) = DATA.obj(1);
        objL(q,2) = DATA.objC(1);
        objL(q,3) = ER.Results.References{q}.solution.objective;
    end
    plot(ER.Simulations,objL(:,1),'Color',Clr(1,:),'linewidth',2)
    plot(ER.Simulations,objL(:,2),'Color',Clr(2,:),'linewidth',2)
    plot(ER.Simulations,objL(:,3),'Color',Clr(4,:),'linewidth',2)
hold off
xlabel('Angle [°]'),ylabel('Objective [N^2]'),title('Energetic Race')
legend('Person','Motor','Person Only','Location','Best')
grid
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(fig,[path,'\Objective_-_',group],'-dpdf','-r0')