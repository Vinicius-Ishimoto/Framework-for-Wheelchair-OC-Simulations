%% MQ 2 Factors

load('EnergeticRace_I2V0.5D1.2.mat');

DATA = fvalid(FDATA.Plano.Reference.Person, FDATA.Plano.Reference.Raw);

% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*FDATA.Plano.Reference.Person.Rr;
u  = interp1(DATA.time,DATA.taup/FDATA.Plano.Reference.Person.R,t,'linear','extrap');

reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*FDATA.Plano.Reference.Person.Rr,'-r','linewidth',2)
	plot(T,Y,'-b','linewidth',2)
hold off
legend('d\theta Sim',['d\theta Ref',char(10),sprintf('M_i = %0.2f',ReferenceModel.Mass),char(10),sprintf('C_i = %0.2f',ReferenceModel.Friction)],'Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

%% MQ 1 Factor

m = FDATA.Plano.Reference.Person.Mf;
reg = [dx(1:end-1)];
thetaf = pinv(reg)*[dx(2:end)-(t(2)-t(1))/m*u(1:end-1)];

ReferenceModel.Mass = m;
ReferenceModel.Friction = (1-thetaf(1,end))*m/(t(2)-t(1));

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*FDATA.Plano.Reference.Person.Rr,'-r','linewidth',2)
	plot(T,Y,'-b','linewidth',2)
hold off
legend('d\theta Sim',['d\theta Ref',char(10),sprintf('M_i = %0.2f',ReferenceModel.Mass),char(10),sprintf('C_i = %0.2f',ReferenceModel.Friction)],'Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

%% Transfer Function extimated

data = iddata(dx,u,t(2)-t(1));
data.InputName  = 'Força';
data.InputUnit  = 'N';
data.OutputName = 'Velocidade';
data.OutputUnit = 'm/s';
data.TimeUnit   = 's';

opt = tfestOptions('InitialCondition','zero');
SYS = tfest(data,1,0,opt);
[NUM,DEN,TS] = tfdata(SYS);
% S = SYS;
ReferenceModel.Mass = 1/NUM{1}(end);
ReferenceModel.Friction = DEN{1}(end)/NUM{1}(end);

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*FDATA.Plano.Reference.Person.Rr,'-r','linewidth',2)
	plot(T,Y,'-b','linewidth',2)
hold off
legend('d\theta Sim',['d\theta Ref',char(10),sprintf('M_i = %0.2f',ReferenceModel.Mass),char(10),sprintf('C_i = %0.2f',ReferenceModel.Friction)],'Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid