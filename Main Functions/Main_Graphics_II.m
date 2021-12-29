%% Reference Simulations

ssr = load(['byRollingResistance_Indice2_SteadyState','.mat']);
path = 'C:\Users\Vinicius\Google Drive\Tese\Artigos\Artigo da dissertação\LaTeX\figures';

ssr.rt = fvalid(ssr.Person.Reference, ssr.Results.Reference);
fgraphicW(ssr.Person.Reference, ssr.rt, 'Movement', true, 'Density', 0.2,'Path', path, 'Name', 'SS_stick_Figure');

str = load(['byRollingResistance_Indice2_EnergeticRace','.mat']);
str.rt = fvalid(str.Person.Reference, str.Results.Reference);
fgraphicW(str.Person.Reference, str.rt, 'Movement', true, 'Density', 0.2,'Path', path, 'Name', 'ST_stick_Figure');

% Least Square
t  = linspace(ssr.rt.time(1),ssr.rt.time(end),500)';
dx = interp1(ssr.rt.time,ssr.rt.dtheta,t,'spline','extrap')*ssr.Person.Reference.Rr;
u  = interp1(ssr.rt.time,ssr.rt.taup/ssr.Person.Reference.R,t,'linear','extrap');
reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);
IReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
IReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);
[~,Y] = ode45(@(tv,vt) (1/IReferenceModel.Mass)*(interp1(t,u,tv)-IReferenceModel.Friction*vt),t,0);
Y = Y+mean(dx-Y);
fit = goodnessOfFit(Y,dx,'NRMSE');
fitOld = -1;
while (abs((fitOld-fit)/fit) > 0.1)
    [T,Y] = ode45(@(tv,vt) (1/IReferenceModel.Mass)*(interp1(t,u,tv)-IReferenceModel.Friction*vt),t,Y(1));
    Y = Y+mean(dx-Y);
    fitOld = fit;
    fit = goodnessOfFit(Y,dx,'NRMSE');
end

fig = figure('Name','Comparison','DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
hold on
	plot(ssr.rt.time,ssr.rt.dtheta*ssr.Person.Reference.Rr,'-r','linewidth',2)
	plot(T,Y,'-b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid
set(fig,'PaperPositionMode','Auto')
print(fig,[path,'\','SS_Comparison'],'-dpdf','-r0')

fprintf(['Steady State & ']);
fprintf('%7.2f & ',IReferenceModel.Mass);
fprintf('%7.2f & ',IReferenceModel.Friction);
fprintf('%7.2f\\\\\n',fit*100);

% Least Square
t  = linspace(str.rt.time(1),str.rt.time(end),500)';
dx = interp1(str.rt.time,str.rt.dtheta,t,'spline','extrap')*str.Person.Reference.Rr;
u  = interp1(str.rt.time,str.rt.taup/str.Person.Reference.R,t,'linear','extrap');
reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);
IReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
IReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);
[T,Y] = ode45(@(tv,vt) (1/IReferenceModel.Mass)*(interp1(t,u,tv)-IReferenceModel.Friction*vt),t,0);
% Y = Y+mean(dx-Y);
fit = goodnessOfFit(Y,dx,'NRMSE');

fig = figure('Name','Comparison','DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
hold on
	plot(str.rt.time,str.rt.dtheta*str.Person.Reference.Rr,'-r','linewidth',2)
	plot(T,Y,'-b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid
set(fig,'PaperPositionMode','Auto')
print(fig,[path,'\','ST_Comparison'],'-dpdf','-r0')

fprintf(['Start-up & ']);
fprintf('%7.2f & ',IReferenceModel.Mass);
fprintf('%7.2f & ',IReferenceModel.Friction);
fprintf('%7.2f\\\\\n',fit*100);