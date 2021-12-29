function Tab = Table_Graphics(PAR,Results,Name,Linear)

%% Data Analysis
DATA = fvalid(PAR, Results);
Toc = strings([1,29]);
Tab = zeros(numel(DATA.time),29);
Sof = '%10.4f';
Toc(1,1) = "time";
Tab(:,1) = DATA.time;

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
Sof = [Sof,' %10.4f',' %10.4f',' %10.4f'];
Toc(1,2:4) = ["alpha", "beta", "theta"];
Tab(:,2:4) = [DATA.alpha*180/pi,DATA.beta*180/pi,DATA.theta*180/pi];

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
Sof = [Sof,' %10.4f',' %10.4f',' %10.4f'];
Toc(1,5:7) = ["dalpha", "dbeta", "dtheta"];
Tab(:,5:7) = [DATA.dalpha,DATA.dbeta,DATA.dtheta];

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
Sof = [Sof,' %10.4f',' %10.4f'];
Toc(1,8:9) = ["fx", "fy"];
Tab(:,8:9) = [DATA.Fx3,DATA.Fy3];

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
Sof = [Sof,' %10.4f',' %10.4f',' %10.4f'];
Toc(1,10:12) = ["tau_s", "tau_e", "tau_p"];
Tab(:,10:12) = [DATA.tau_s,DATA.tau_e,DATA.taup];

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
Tab(:,13:16) = [DATA.tau_se,DATA.tau_sf,DATA.tau_ee,DATA.tau_ef];
if isfield(DATA,'tau_sp')
    Tab(:,25:26) = [DATA.tau_sp,DATA.tau_ep];
end
end
Sof = [Sof,' %10.4f',' %10.4f',' %10.4f',' %10.4f'];
Toc(1,13:16) = ["tau_se", "tau_sf", "tau_ee", "tau_ef"];
Sof = [Sof,' %10.4f',' %10.4f'];
Toc(1,25:26) = ["tau_sp", "tau_ep"];

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
Tab(:,17:20) = [DATA.act_se,DATA.act_sf,DATA.act_ee,DATA.act_ef];
end
Sof = [Sof,' %10.4f',' %10.4f',' %10.4f',' %10.4f'];
Toc(1,17:20) = ["act_se", "act_sf", "act_ee", "act_ef"];

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
    Tab(:,27:28) = [DATA.dveli,DATA.dtheta*PAR.Rr];
	
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
    Tab(:,29) = PAR.rt*PAR.Kt*DATA.imotor;
end
Sof = [Sof,' %10.4f',' %10.4f'];
Toc(1,27:28) = ["v_imp", "v_re"];
Sof = [Sof,' %10.4f'];
Toc(1,29) = "tau_m";

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
Sof = [Sof, ' %10.4f',' %10.4f'];
Toc(1,21:22) = ["arox", "aroy"];
Tab(2:21,21:22) = [xunit2,yunit2];
Sof = [Sof, ' %10.4f',' %10.4f'];
Toc(1,23:24) = ["trac_x", "trac_y"];
Tab(:,23:24) = [xi,yi];

fileID = fopen([Name,'.txt'],'w');
fprintf(fileID,' %10s',Toc);
fprintf(fileID,'\r\n');
for i=1:size(Tab,1)
     fprintf(fileID,[Sof,'\r\n'], Tab(i,:));
end
fclose(fileID);

%% Final Table Data

lint = "Data";
lin1 = "Speed (m/s)";
lin2 = "Distance traveled (m)";
lin3 = "Stroke cycle time (s)";
lin4 = "Push phase (\\%%)";
lin5 = "Stroke cycle frequency (Hz)";
lin6 = "Range of Motion-shoulder ($^o$)";
lin7 = "Range of Motion-elbow ($^o$)";
lin8 = "Push angle ($^o$)";
lin9 = "Max tangencial force (N)";
for i=1:floor(numel(Results.solution.phase)/2)
    timeP = Results.solution.phase(2*i-1).time;
    alphP = Results.solution.phase(2*i-1).control(:,3);
    betaP = Results.solution.phase(2*i-1).control(:,4);
    thetP = Results.solution.phase(2*i-1).position(:,1);
    taupP = Results.solution.phase(2*i-1).taup;
    % dthtP = Results.solution.phase(2*i-1).velocity(:,1);
    timeR = Results.solution.phase(2*i).time;
    alphR = Results.solution.phase(2*i).position(:,1);
    betaR = Results.solution.phase(2*i).position(:,2);
    thetR = Results.solution.phase(2*i).position(:,3);
    
    lint = lint + " & cycle " + num2str(i);
    lin1 = lin1 + " & " + num2str(PAR.Rr*(thetR(end)-thetP(1))/(timeR(end)-timeP(1)),'%.2f');
    lin2 = lin2 + " & " + num2str(PAR.Rr*(thetR(end)-thetP(1)),'%.2f');
    lin3 = lin3 + " & " + num2str(timeR(end)-timeP(1),'%.2f');
    lin4 = lin4 + " & " + num2str(100*(timeP(end)-timeP(1))/(timeR(end)-timeP(1)),'%.2f');
    lin5 = lin5 + " & " + num2str(1/(timeR(end)-timeP(1)),'%.2f');
    ap = [-alphP;-alphR]*180/pi;
    bt = [alphP-betaP;alphR-betaR]*180/pi;
    lin6 = lin6 + " & " + num2str(max(ap)-min(ap),'%.2f');
    lin7 = lin7 + " & " + num2str(max(bt)-min(bt),'%.2f');
    lin8 = lin8 + " & " + num2str((thetP(end)-thetP(1))*180/pi,'%.2f');
    lin9 = lin9 + " & " + num2str(max(abs(taupP)/PAR.R),'%.2f');
end

fprintf(lint+"\\\\\n");
fprintf("\\hline\n");
fprintf(lin1+"\\\\\n");
fprintf(lin2+"\\\\\n");
fprintf(lin3+"\\\\\n");
fprintf(lin4+"\\\\\n");
fprintf(lin5+"\\\\\n");
fprintf(lin6+"\\\\\n");
fprintf(lin7+"\\\\\n");
fprintf(lin8+"\\\\\n");
fprintf(lin9+"\\\\\n");