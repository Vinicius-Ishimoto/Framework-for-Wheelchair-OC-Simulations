% Simulações com Controle


%% Simulation data (1) Steady-State
load([cd,'\Results\Steady State\','Reference_SteadyState','.mat']);
Speed = Results.Options.MeanVelocity;
Max_Shoulder = Results.Options.MaxShoulder; % Maximum torque in shoulder
Max_Elbow = Results.Options.MaxElbow; % Maximum torque in elbow
Person.PAR.Mi = ReferenceModel.Mass;
Person.PAR.Ci = ReferenceModel.Friction;
Guess = multi_cycle_PID_integrate(Person.PAR, Results);
Options = {'Max_shoulder',Max_Shoulder,'Max_elbow',Max_Elbow,'Guess',Guess};
Results = steady_state_PID_controller(Person.PAR, Speed, Options{:});

%% Data Analysis
DATA = fvalid(Person.PAR, Results);

% Phases interfaces
PhasesInt = 1;
for i=1:numel(Results.solution.phase)
	if i==1
		PhasesInt = [PhasesInt, numel(Results.solution.phase(i).time)];
	else
		PhasesInt = [PhasesInt, numel(Results.solution.phase(i).time)-1+PhasesInt(end)];
	end
end

fig = figure('Name','Angulos','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.alpha*180/pi,'-r','linewidth',2)
	plot(DATA.time,DATA.beta*180/pi,'-m','linewidth',2)
	plot(DATA.time,DATA.theta*180/pi,'-b','linewidth',2)
hold off
legend('\alpha','\beta','\theta','Location','Best')
xlabel('tempo [s]'),ylabel('angle [°]'),title('Ângulos')
grid

fig = figure('Name','Velocidades','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dalpha,'-r','linewidth',2)
	plot(DATA.time,DATA.dbeta,'-m','linewidth',2)
	plot(DATA.time,DATA.dtheta,'-b','linewidth',2)
hold off
legend('d\alpha','d\beta','d\theta','Location','Best')
xlabel('tempo [s]'),ylabel('angular velocity [rad/s]'),title('Velocidade Angular')
grid

fig = figure('Name','Forças de Contato','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.Fx3,'-r','linewidth',2)
	plot(DATA.time,DATA.Fy3,'-b','linewidth',2)
hold off
legend('Fx','Fy','Location','Best')
xlabel('tempo [s]'),ylabel('forces [N]'),title('Forças de Contato')
grid

fig = figure('Name','Torque','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.tau_s,'-r','linewidth',2)
	plot(DATA.time,DATA.tau_e,'-b','linewidth',2)
hold off
legend('\tau_e','\tau_s','Location','Best')
xlabel('tempo [s]'),ylabel('forces [N]'),title('Torques')
grid

fig = figure('Name','Velocity Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*Person.PAR.Rr,'-r','linewidth',2)
	plot(DATA.time,DATA.dveli,'-b','linewidth',2)
hold off
legend('v_{real}','v_{imp}','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [m/s]'),title('Velocidades')
grid

fig = figure('Name','Applied Torques','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.taup,'-r','linewidth',2)
	plot(DATA.time,Person.PAR.rt*Person.PAR.Kt*DATA.imotor,'-b','linewidth',2)
hold off
legend('\tau_p','\tau_m','Location','Best')
xlabel('tempo [s]'),ylabel('Torques [Nm]'),title('Torques Aplicados')
grid

xi = -Person.PAR.h+Person.PAR.A*cos(DATA.alpha)+Person.PAR.B*cos(DATA.beta);
yi = Person.PAR.Y-Person.PAR.A*sin(DATA.alpha)-Person.PAR.B*sin(DATA.beta);

figure('Name','Perfil da mão');
hold on
    th = linspace(Person.PAR.thetac1,Person.PAR.thetac2,20)';
    xunit2 = Person.PAR.R * cos(th);
    yunit2 = -Person.PAR.R * sin(th);
	for i=1:numel(PhasesInt)-1
		if rem(i,2)
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'*-b')
		else
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'*-r')
		end
	end
	plot(xunit2,yunit2,'*-m')
    plot([0;Person.PAR.R*cos(Person.PAR.thetac2)],[0;-Person.PAR.R*sin(Person.PAR.thetac2)],'.-m',[0;Person.PAR.R*cos(Person.PAR.thetac1)],[0;-Person.PAR.R*sin(Person.PAR.thetac1)],'.-m', ...
        'linewidth',2)
hold off
axis('equal')
legend('\tau_s sim','\tau_e sim','pushrim exp','Location','best')
grid

% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*Person.PAR.Rr;
u  = interp1(DATA.time,DATA.taup/Person.PAR.R,t,'linear','extrap');

reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*Person.PAR.Rr,'-r','linewidth',2)
	plot(T,Y+mean(dx-Y),'-b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

%% Save Simulation
save([cd,'\Results\','Control_SteadyState','.mat'],'Results','Person','ReferenceModel','Options')

%% Simulation data (2) Energetic Race
load([cd,'\Results\','Energetic Race\Reference\indice_2\','Reference_EnergeticRace','.mat']);
Speed = Results.Options.MeanVelocity;
Num_Cycles = Results.Options.NumCycles;
Distance = Results.Options.TotalDistance;
Max_Shoulder = Results.Options.MaxShoulder; % Maximum torque in shoulder
Max_Elbow = Results.Options.MaxElbow; % Maximum torque in elbow
Person.PAR.Mi = ReferenceModel.Mass;
Person.PAR.Ci = ReferenceModel.Friction;
Guess = multi_cycle_PID_integrate(Person.PAR, Results);
%%
Options = {'Max_shoulder',Max_Shoulder, 'Max_elbow',Max_Elbow, 'Contact_angle', 10, 'Guess', Guess,'indice',Results.Options.Indice};
Results = multi_cycle_PID_controller(Person.PAR, Num_Cycles, Speed, Distance, Options{:});

%% Data Analysis
DATA = fvalid(Person.PAR, Results);

% Phases interfaces
PhasesInt = 1;
for i=1:numel(Results.solution.phase)
	if i==1
		PhasesInt = [PhasesInt, numel(Results.solution.phase(i).time)];
	else
		PhasesInt = [PhasesInt, numel(Results.solution.phase(i).time)-1+PhasesInt(end)];
	end
end

fig = figure('Name','Angulos','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.alpha*180/pi,'-r','linewidth',2)
	plot(DATA.time,DATA.beta*180/pi,'-m','linewidth',2)
	plot(DATA.time,DATA.theta*180/pi,'-b','linewidth',2)
	for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('\alpha','\beta','\theta','Location','Best')
xlabel('tempo [s]'),ylabel('angle [°]'),title('Ângulos')
grid

fig = figure('Name','Velocidades','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dalpha,'-r','linewidth',2)
	plot(DATA.time,DATA.dbeta,'-m','linewidth',2)
	plot(DATA.time,DATA.dtheta,'-b','linewidth',2)
	for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('d\alpha','d\beta','d\theta','Location','Best')
xlabel('tempo [s]'),ylabel('angular velocity [rad/s]'),title('Velocidade Angular')
grid

fig = figure('Name','Forças de Contato','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.Fx3,'-r','linewidth',2)
	plot(DATA.time,DATA.Fy3,'-b','linewidth',2)
	for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('Fx','Fy','Location','Best')
xlabel('tempo [s]'),ylabel('forces [N]'),title('Forças de Contato')
grid

fig = figure('Name','Torque','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.tau_s,'-r','linewidth',2)
	plot(DATA.time,DATA.tau_e,'-b','linewidth',2)
	for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('\tau_e','\tau_s','Location','Best')
xlabel('tempo [s]'),ylabel('forces [N]'),title('Torques')
grid

fig = figure('Name','Velocity Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dveli,'-b','linewidth',2)
	plot(DATA.time,DATA.dtheta*Person.PAR.Rr,'-r','linewidth',2)
	for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend(['v_{imp}',char(10),sprintf('M_i = %0.2f',Results.Options.ImpedanceMass),char(10),sprintf('C_i = %0.2f',Results.Options.ImpedanceFriction)],'v_{real}','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [m/s]'),title('Velocidades')
grid

fig = figure('Name','Applied Torques','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.taup,'-r','linewidth',2)
	plot(DATA.time,Person.PAR.rt*Person.PAR.Kt*DATA.imotor,'-b','linewidth',2)
	for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('\tau_p','\tau_m','Location','Best')
xlabel('tempo [s]'),ylabel('Torques [Nm]'),title('Torques Aplicados')
grid

% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*Person.PAR.Rr;
u  = interp1(DATA.time,DATA.taup/Person.PAR.R,t,'linear','extrap');

reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

ReferenceModel.Mass = (t(2)-t(1))/thetaf(2,end);
ReferenceModel.Friction = (1-thetaf(1,end))/thetaf(2,end);

xi = -Person.PAR.h+Person.PAR.A*cos(DATA.alpha)+Person.PAR.B*cos(DATA.beta);
yi = Person.PAR.Y-Person.PAR.A*sin(DATA.alpha)-Person.PAR.B*sin(DATA.beta);

figure('Name','Perfil da mão');
hold on
    th = linspace(Person.PAR.thetac1,Person.PAR.thetac2,20)';
    xunit2 = Person.PAR.R * cos(th);
    yunit2 = -Person.PAR.R * sin(th);
	for i=1:numel(PhasesInt)-1
		if rem(i,2)
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'*-b')
		else
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'*-r')
		end
	end
	plot(xunit2,yunit2,'*-m')
    plot([0;Person.PAR.R*cos(Person.PAR.thetac2)],[0;-Person.PAR.R*sin(Person.PAR.thetac2)],'.-m',[0;Person.PAR.R*cos(Person.PAR.thetac1)],[0;-Person.PAR.R*sin(Person.PAR.thetac1)],'.-m', ...
        'linewidth',2)
hold off
axis('equal')
legend('\tau_s sim','\tau_e sim','pushrim exp','Location','best')
grid

[T,Y] = ode45(@(tv,vt) (1/ReferenceModel.Mass)*(interp1(t,u,tv)-ReferenceModel.Friction*vt),t,0);
fig = figure('Name','Comparison','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.dtheta*Person.PAR.Rr,'-r','linewidth',2)
	plot(T,Y+mean(dx-Y),'-b','linewidth',2)
hold off
legend('d\theta Sim','d\theta Ref','Location','Best')
xlabel('tempo [s]'),ylabel('velocity [rad/s]'),title('Comparison')
grid

%% Save Simulation
save([cd,'\Results\','Control_EnergeticRace','.mat'],'Results','Person','ReferenceModel','Options')