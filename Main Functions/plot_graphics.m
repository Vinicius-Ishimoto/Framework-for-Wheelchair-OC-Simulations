function plot_graphics(PAR,Results,Name,Path,varargin)

if nargin < 4
    plots = false;
else
    plots = true;
end



if nargin < 3
    Name = '_';
end

p = inputParser;

addParameter(p,'Format','pdf',@ischar);
% addParameter(p,'isRef',false,@islogical);
parse(p,varargin{:});
Format = p.Results.Format;

%% Data Analysis
DATA = fvalid(PAR, Results);

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
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
if plots
    print(fig,[Path,'\',Name,' - angles'],['-d',Format],'-r0')
end

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
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
if plots
    print(fig,[Path,'\',Name,' - angular_velocities'],['-d',Format],'-r0')
end
    
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
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
if plots
    print(fig,[Path,'\',Name,' - contact_forces'],['-d',Format],'-r0')
end

fig = figure('Name',['Torque - ',Name],'DefaultAxesFontSize',12);
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
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
if plots
    print(fig,[Path,'\',Name,' - torques'],['-d',Format],'-r0')
end
    
if isfield(Results.solution.phase(1),'state')
	fig = figure('Name',['Velocity Comparison - ',Name],'DefaultAxesFontSize',12);
	hold on
		plot(DATA.time,DATA.dveli,'-b','linewidth',2)
		plot(DATA.time,DATA.dtheta*PAR.Rr,'-r','linewidth',2)
		for i=2:numel(PhasesInt)-1
			plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
		end
	hold off
	legend(['v_{imp}',char(10),sprintf('M_i = %0.2f',Results.Options.ImpedanceMass),char(10),sprintf('C_i = %0.2f',Results.Options.ImpedanceFriction)],'v_{real}','Location','Best')
	xlabel('tempo [s]'),ylabel('velocity [m/s]'),title('Velocidades')
	grid
    set(fig,'Units','Inches');
    pos = get(fig,'Position');
    set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
        % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
    % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
    if plots
        print(fig,[Path,'\',Name,' - velocity_comparison'],['-d',Format],'-r0')
    end
    
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
    set(fig,'Units','Inches');
    pos = get(fig,'Position');
    set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
    % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
    if plots
        print(fig,[Path,'\',Name,' - applied_torques'],['-d',Format],'-r0')
    end
else
    fig = figure('Name',['Applied Torques - ',Name],'DefaultAxesFontSize',12);
	hold on
		plot(DATA.time,DATA.taup,'-r','linewidth',2)
		for i=2:numel(PhasesInt)-1
			plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
		end
	hold off
	legend('\tau_p','Location','Best')
	xlabel('tempo [s]'),ylabel('Torques [Nm]'),title('Torques Aplicados')
	grid
    set(fig,'Units','Inches');
    pos = get(fig,'Position');
    set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
    % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
    if plots
        print(fig,[Path,'\',Name,' - applied_torques'],['-d',Format],'-r0')
    end
end

xi = -PAR.h+PAR.A*cos(DATA.alpha)+PAR.B*cos(DATA.beta);
yi = PAR.Y-PAR.A*sin(DATA.alpha)-PAR.B*sin(DATA.beta);

fig = figure('Name',['Perfil da mão - ',Name],'DefaultAxesFontSize',12);
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
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
if plots
    print(fig,[Path,'\',Name,' - hand_patern'],['-d',Format],'-r0')
end