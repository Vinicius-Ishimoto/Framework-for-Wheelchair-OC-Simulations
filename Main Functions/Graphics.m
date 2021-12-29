function Graphics(PAR,Results,Name,Linear)

%% Data Analysis
DATA = fvalid(PAR, Results);

if (nargin < 4)
    Linear = false;
end

% Phases interfaces
PhasesInt = 1;
for i=1:numel(Results.solution.phase)
	if i==1
		PhasesInt = [PhasesInt, numel(Results.solution.phase(i).time)];
	else
		PhasesInt = [PhasesInt, numel(Results.solution.phase(i).time)-1+PhasesInt(end)];
	end
end

fig = figure('Name',['Angulos - ',Name],'DefaultAxesFontSize',12);
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

fig = figure('Name',['Velocidades - ',Name],'DefaultAxesFontSize',12);
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

fig = figure('Name',['Forças de Contato - ',Name],'DefaultAxesFontSize',12);
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

fig = figure('Name',['Torque - ',Name],'DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.tau_s,'-r','linewidth',2)
	plot(DATA.time,DATA.tau_e,'-b','linewidth',2)
    plot(DATA.time,DATA.taup,'.-k','linewidth',2)
	for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('\tau_e','\tau_s','\tau_t','Location','Best')
xlabel('tempo [s]'),ylabel('torque [Nm]'),title('Torques')
grid

if isfield(DATA,'tau_se')
    fig = figure('Name','Ativacoes','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.tau_se,'-r','linewidth',2)
	plot(DATA.time,DATA.tau_sf,'.m','linewidth',2)
	plot(DATA.time,DATA.tau_ee,'-b','linewidth',2)
    plot(DATA.time,DATA.tau_ef,'.g','linewidth',2)
    for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('\tau_{se}','\tau_{sf}','\tau_{ee}','\tau_{ef}','Location','Best')
xlabel('tempo [s]'),ylabel('torques [Nm]'),title('Torques (detalhado)')
grid
end

if isfield(DATA,'act_se')
    fig = figure('Name','Ativacoes','DefaultAxesFontSize',12);
hold on
	plot(DATA.time,DATA.act_se,'-r','linewidth',2)
	plot(DATA.time,DATA.act_sf,'.m','linewidth',2)
	plot(DATA.time,DATA.act_ee,'-b','linewidth',2)
    plot(DATA.time,DATA.act_ef,'.g','linewidth',2)
    for i=2:numel(PhasesInt)-1
		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
	end
hold off
legend('A_{se}','A_{sf}','A_{ee}','A_{ef}','Location','Best')
xlabel('tempo [s]'),ylabel('activation []'),title('Ativacoes')
grid
end

if isfield(Results.Options,'ImpedanceMass')

if Linear
% Least Square
t  = linspace(DATA.time(1),DATA.time(end),500)';
dx = interp1(DATA.time,DATA.dtheta,t,'spline','extrap')*PAR.Rr;
% dx = interp1(DATA.time,DATA.dveli,t,'spline','extrap');
u  = interp1(DATA.time,DATA.taup/PAR.R,t,'linear','extrap');

reg = [dx(1:end-1),u(1:end-1)];
thetaf = pinv(reg)*dx(2:end);

LinearModel.Mass = (t(2)-t(1))/thetaf(2,end);
LinearModel.Friction = (1-thetaf(1,end))/thetaf(2,end);

[T,Y] = ode45(@(tv,vt) (1/LinearModel.Mass)*(interp1(t,u,tv)-LinearModel.Friction*vt),t,dx(1));
end

	fig = figure('Name',['Velocity Comparison - ',Name],'DefaultAxesFontSize',12);
	hold on
        if isfield(DATA,'dveli')
            plot(DATA.time,DATA.dveli,'-b','linewidth',2)
        end
		plot(DATA.time,DATA.dtheta*PAR.Rr,'-r','linewidth',2)
        if Linear
        plot(T,Y+mean(dx-Y),'-m','linewidth',2)
        end
		for i=2:numel(PhasesInt)-1
			plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
		end
	hold off
    if Linear
	legend(['v_{imp}',char(10),sprintf('M_i = %0.2f',Results.Options.ImpedanceMass),char(10),sprintf('C_i = %0.2f',Results.Options.ImpedanceFriction)],'v_{real}',['v_{est}',char(10),sprintf('M_e = %0.2f',LinearModel.Mass),char(10),sprintf('C_e = %0.2f',LinearModel.Friction)],'Location','Best')
    else
    legend(['v_{imp}',char(10),sprintf('M_i = %0.2f',Results.Options.ImpedanceMass),char(10),sprintf('C_i = %0.2f',Results.Options.ImpedanceFriction)],'v_{real}','Location','Best')   
    end
    xlabel('tempo [s]'),ylabel('velocity [m/s]'),title('Velocidades')
	grid
	
	fig = figure('Name',['Applied Torques - ',Name],'DefaultAxesFontSize',12);
	hold on
		plot(DATA.time,DATA.taup,'-r','linewidth',2)
		plot(DATA.time,PAR.rt*PAR.Kt*DATA.imotor,'-b','linewidth',2)
		for i=2:numel(PhasesInt)-1
			plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
		end
	hold off
	legend('\tau_p','\tau_m','Location','Best')
	xlabel('tempo [s]'),ylabel('Torques [Nm]'),title('Torques Aplicados')
	grid
end

xi = -PAR.h+PAR.A*cos(DATA.alpha)+PAR.B*cos(DATA.beta);
yi = PAR.Y-PAR.A*sin(DATA.alpha)-PAR.B*sin(DATA.beta);

figure('Name',['Perfil da mão - ',Name]);
hold on
    th = linspace(PAR.thetac1,PAR.thetac2,20)';
    xunit2 = PAR.R * cos(th);
    yunit2 = -PAR.R * sin(th);
	for i=1:numel(PhasesInt)-1
		if rem(i,2)
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'*-b')
		else
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'*-r')
		end
	end
	plot(xunit2,yunit2,'*-m')
    plot([0;PAR.R*cos(PAR.thetac2)],[0;-PAR.R*sin(PAR.thetac2)],'.-m',[0;PAR.R*cos(PAR.thetac1)],[0;-PAR.R*sin(PAR.thetac1)],'.-m', ...
        'linewidth',2)
hold off
axis('equal')
legend('\tau_s sim','\tau_e sim','pushrim exp','Location','best')
grid