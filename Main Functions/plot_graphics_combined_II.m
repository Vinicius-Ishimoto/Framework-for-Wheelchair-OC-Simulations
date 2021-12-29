function plot_graphics_combined_II(DATA,Name,Path,dPath,SimuCases,Format)

if nargin < 6
    Format = '-dpdf';
else
    switch Format
        case 'eps'
            Format = '-depsc';
    end
end

if nargin < 4
    plots = false;
else
    plots = true;
end

if nargin < 3
    Name = '_';
end

Clr = [0, 0, 0;
    1, 0, 0;
    1, 0, 1;
    0, 0, 1;
    0, 1, 1;
    0, 1, 0];

numMr = 10;
stepMr = floor(numel(DATA.PIController{1}.Results.time)/numMr/5);
Mrk = {'-';'-';'--';':';'-.';'--'};
% Mrk = {'-o';'-+';'--x';':s';'-.d';'--^'};

posMr{1} = floor(linspace(1,numel(DATA.Reference.Results.time),numMr));
posMr{2} = floor(linspace(1+1*stepMr,numel(DATA.PIController{1}.Results.time),numMr));
posMr{3} = floor(linspace(1+2*stepMr,numel(DATA.PIController{2}.Results.time),numMr));
posMr{4} = floor(linspace(1+3*stepMr,numel(DATA.PIController{3}.Results.time),numMr));
posMr{5} = floor(linspace(1+4*stepMr,numel(DATA.PIController{4}.Results.time),numMr));

for i=1:5
    figOpt{i} = {Mrk{i},'Color',Clr(i,:),'linewidth',2};
%     figOpt{i} = {Mrk{i},'MarkerIndices',posMr{i},'Color',Clr(i,:),'linewidth',1.5};
end

iClr = 1;

Lgd = {'Reference'};

%% Phases interfaces
PhasesInt = 1;
for i=1:numel(DATA.Reference.Raw.solution.phase)
	if i==1
		PhasesInt = [PhasesInt, numel(DATA.Reference.Raw.solution.phase(i).time)];
	else
		PhasesInt = [PhasesInt, numel(DATA.Reference.Raw.solution.phase(i).time)-1+PhasesInt(end)];
	end
end

%% Graphic

fig = figure('Name',['Velocities - ',Name],'DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
%     DATA = fvalid(PAR.Reference, Results.Reference);
%     Mki = ceil(linspace(1,length(DATA.time),20));
%     h(1,1) = plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Reference.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%     set(h(1,1),'Visible','off')
    plot(DATA.Reference.Results.time,DATA.Reference.Results.dtheta*DATA.Reference.Person.Rr,figOpt{iClr}{:})
%     plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Reference.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
%         DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
%         Mki = ceil(linspace((1+q),length(DATA.time),20));
%         h(1+q,1) = plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%         set(h(1+q,1),'Visible','off')
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.dtheta*DATA.PIController{q}.Person.Rr,figOpt{iClr}{:})
%         plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
        Lgd{iClr} = {['M_{',num2str(SimuCases(q,1)*100),'}-C_{',num2str(SimuCases(q,2)*100),'}']};
        iClr = iClr + 1;
    end
    
%     % Phases Interface
%     for i=2:numel(PhasesInt)-1
%         plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
%     end
hold off
xlabel('time [s]'),ylabel('Velocity [m/s]'),title('Longitudinal Velocity')
% legend(h,[Lgd{:}],'Location','Best')
legend([Lgd{:}],'Location','Best')
grid, box on
set(fig,'PaperPositionMode','Auto')
if plots
    print(fig,[Path,'\',Name,'_-_Velocity'],Format,'-r0')
end


clear h
fig = figure('Name',['Torques - ',Name],'DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
set(gca, 'FontName', 'Times New Roman')
iClr = 1;
hold on
    % Data Analysis (Reference)
%     DATA = fvalid(PAR.Reference, Results.Reference);
%     Mki = ceil(linspace(1,length(DATA.time),40));
%     h(1,1) = plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Reference.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%     set(h(1,1),'Visible','off')
    plot(DATA.Reference.Results.time,DATA.Reference.Results.taup,figOpt{iClr}{:})
%     plot(DATA.Reference.Results.time,DATA.Reference.Results.taup/DATA.Reference.Person.R,'Color',Clr(iClr,:),'linewidth',2)
%     plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Reference.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
%         DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
%         Mki = ceil(linspace((1+q),length(DATA.time),40));
%         h(1+q,1) = plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Controller{q}.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%         set(h(1+q,1),'Visible','off')
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.taup,figOpt{iClr}{:})
%         plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.taup/DATA.PIController{q}.Person.R,'Color',Clr(iClr,:),'linewidth',2)
%         plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Controller{q}.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
        Lgd{iClr} = {['M_{',num2str(SimuCases(q,1)*100),'}-C_{',num2str(SimuCases(q,2)*100),'}']};
        iClr = iClr + 1;
    end
    
%     % Phases Interface
%     for i=2:numel(PhasesInt)-1
%         plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
%     end
hold off
xlabel('Time [s]'),ylabel('Torque�[Nm]'),title('User''s Propulsion Torque')
% legend(h,[Lgd{:}],'Location','Best')
legend([Lgd{:}],'Location','Best')
grid, box on
set(fig,'PaperPositionMode','Auto')
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s\v1.1
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures\results
if plots
    print(fig,[Path,'\',Name,'_-_TForce'],Format,'-r0')
end

clear h
ymax = max([DATA.Reference.Results.tau_sf,DATA.Reference.Results.tau_se,DATA.Reference.Results.tau_ee,DATA.Reference.Results.tau_ef]);
for q=1:numel(DATA.PIController)
    ymax = max([ymax;DATA.PIController{q}.Results.tau_sf,DATA.PIController{q}.Results.tau_se,DATA.PIController{q}.Results.tau_ee,DATA.PIController{q}.Results.tau_ef]);
end
ymax = max(ymax)+1;
fig = figure('Name',['Torques Detailled - ',Name],'DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
subplot(2,2,1)
iClr = 1;
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.tau_sf,figOpt{iClr}{:})
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.tau_sf,figOpt{iClr}{:})
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Torque�[Nm]'),title('A) Shoulder Flexion')
grid, box on
ylim([0,ymax])
subplot(2,2,2)
iClr = 1;
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.tau_se,figOpt{iClr}{:})
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.tau_se,figOpt{iClr}{:})
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Torque�[Nm]'),title('B) Shoulder Extension')
grid, box on
ylim([0,ymax])
subplot(2,2,3)
iClr = 1;
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.tau_ef,figOpt{iClr}{:})
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.tau_ef,figOpt{iClr}{:})
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Torque�[Nm]'),title('C) Elbow Flexion')
grid, box on
ylim([0,ymax])
subplot(2,2,4)
set(gca, 'FontName', 'Times New Roman')
iClr = 1;
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.tau_ee,figOpt{iClr}{:})
    Lgd{iClr} = {'Reference'};
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.tau_ee,figOpt{iClr}{:})
        Lgd{iClr} = {['M_{',num2str(SimuCases(q,1)*100),'}-C_{',num2str(SimuCases(q,2)*100),'}']};
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Torque�[Nm]'),title('D) Elbow Extension')
leg = legend([Lgd{:}],'Location','Best');
% leg.NumColumns = 2;
grid, box on
ylim([0,ymax])
set(fig,'PaperPositionMode','Auto')
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s\v1.1
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures\results
if plots
    print(fig,[Path,'\',Name,'_-_PersonTorque'],Format,'-r0')
end

clear h
ymax = max([DATA.Reference.Results.act_sf,DATA.Reference.Results.act_se,DATA.Reference.Results.act_ee,DATA.Reference.Results.act_ef]);
for q=1:numel(DATA.PIController)
    ymax = max([ymax;DATA.PIController{q}.Results.act_sf,DATA.PIController{q}.Results.act_se,DATA.PIController{q}.Results.act_ee,DATA.PIController{q}.Results.act_ef]);
end
ymax = max(ymax)+0.02;
fig = figure('Name',['Activation Detailled - ',Name],'DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
subplot(2,2,1)
iClr = 1;
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.act_sf,figOpt{iClr}{:})
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.act_sf,figOpt{iClr}{:})
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Activation'),title('A) Shoulder Flexion')
ylim([0,ymax])
grid, box on
subplot(2,2,2)
iClr = 1;
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.act_se,figOpt{iClr}{:})
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.act_se,figOpt{iClr}{:})
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Activation'),title('B) Shoulder Extension')
ylim([0,ymax])
grid, box on
subplot(2,2,3)
iClr = 1;
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.act_ef,figOpt{iClr}{:})
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.act_ef,figOpt{iClr}{:})
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Activation'),title('C) Elbow Flexion')
ylim([0,ymax])
grid, box on
subplot(2,2,4)
set(gca, 'FontName', 'Times New Roman')
iClr = 1;
hold on
    % Data Analysis (Reference)
    plot(DATA.Reference.Results.time,DATA.Reference.Results.act_ee,figOpt{iClr}{:})
    Lgd{iClr} = {'Reference'};
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
        plot(DATA.PIController{q}.Results.time,DATA.PIController{q}.Results.act_ee,figOpt{iClr}{:})
        Lgd{iClr} = {['M_{',num2str(SimuCases(q,1)*100),'}-C_{',num2str(SimuCases(q,2)*100),'}']};
        iClr = iClr + 1;
    end
hold off
xlabel('Time [s]'),ylabel('Activation'),title('D) Elbow Extension')
ylim([0,ymax])
leg = legend([Lgd{:}],'Location','Best');
% leg.NumColumns = 2;
grid, box on
set(fig,'PaperPositionMode','Auto')
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s\v1.1
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures\results
if plots
    print(fig,[Path,'\',Name,'_-_PersonActiv'],Format,'-r0')
end

clear h
fig = figure('Name',['Person''s Torques - ',Name],'DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
set(gca, 'FontName', 'Times New Roman')
iClr = 1;
hold on
    % Data Analysis (Reference)
%     DATA = fvalid(PAR.Reference, Results.Reference);
%     Kmotor = PAR.Reference.rt*PAR.Reference.Kt;
%     plot(DATA.time,Kmotor*DATA.imotor/PAR.Reference.Rr,'Color',Clr(iClr,:),'linewidth',2)
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
%         DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
        Kmotor = DATA.PIController{q}.Person.rt*DATA.PIController{q}.Person.Kt;
%         Mki = ceil(linspace((1+q),length(DATA.time),30));
%         h(q,1) = plot(DATA.time(Mki),Kmotor*DATA.imotor(Mki)/PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%         set(h(q,1),'Visible','off')
        plot(DATA.PIController{q}.Results.time,Kmotor*DATA.PIController{q}.Results.imotor,figOpt{iClr}{:})
%         plot(DATA.PIController{q}.Results.time,Kmotor*DATA.PIController{q}.Results.imotor/DATA.PIController{q}.Person.Rr,'Color',Clr(iClr,:),'linewidth',2)
%         plot(DATA.time(Mki),Kmotor*DATA.imotor(Mki)/PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
        Lgd{iClr-1} = {['M_{',num2str(SimuCases(q,1)*100),'}-C_{',num2str(SimuCases(q,2)*100),'}']};
        iClr = iClr + 1;
    end
    
%     % Phases
%     Interface
%     for i=2:numel(PhasesInt)-1
%         plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
%     end
hold off
xlabel('Time [s]'),ylabel('Torque�[Nm]'),title('Motor Torque')
% legend(h,[Lgd{:}],'Location','Best')
legend([Lgd{:}],'Location','Best')
grid, box on
set(fig,'PaperPositionMode','Auto')
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s\v1.1
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures\results
if plots
    print(fig,[Path,'\',Name,'_-_MotorForce'],Format,'-r0')
end

% fig = figure('Name',['Angulos - ',Name],'DefaultAxesFontSize',12);
% hold on
% 	plot(DATA.time,DATA.alpha*180/pi,'-r','linewidth',2)
% 	plot(DATA.time,DATA.beta*180/pi,'-m','linewidth',2)
% 	plot(DATA.time,DATA.theta*180/pi,'-b','linewidth',2)
% 	for i=2:numel(PhasesInt)-1
% 		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
% 	end
% hold off
% legend('\alpha','\beta','\theta','Location','Best')
% xlabel('tempo [s]'),ylabel('angle [°]'),title('Ângulos')
% grid
% set(fig,'Units','Inches');
% pos = get(fig,'Position');
% set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
% if plots
%     print(fig,[Path,'\',Name,' - angles'],'-dpng','-r0')
% end
% 
% fig = figure('Name',['Velocidades - ',Name],'DefaultAxesFontSize',12);
% hold on
% 	plot(DATA.time,DATA.dalpha,'-r','linewidth',2)
% 	plot(DATA.time,DATA.dbeta,'-m','linewidth',2)
% 	plot(DATA.time,DATA.dtheta,'-b','linewidth',2)
% 	for i=2:numel(PhasesInt)-1
% 		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
% 	end
% hold off
% legend('d\alpha','d\beta','d\theta','Location','Best')
% xlabel('tempo [s]'),ylabel('angular velocity [rad/s]'),title('Velocidade Angular')
% grid
% set(fig,'Units','Inches');
% pos = get(fig,'Position');
% set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
% if plots
%     print(fig,[Path,'\',Name,' - angular_velocities'],'-dpng','-r0')
% end
%     
% fig = figure('Name',['Forças de Contato - ',Name],'DefaultAxesFontSize',12);
% hold on
% 	plot(DATA.time,DATA.Fx3,'-r','linewidth',2)
% 	plot(DATA.time,DATA.Fy3,'-b','linewidth',2)
% 	for i=2:numel(PhasesInt)-1
% 		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
% 	end
% hold off
% legend('Fx','Fy','Location','Best')
% xlabel('tempo [s]'),ylabel('forces [N]'),title('Forças de Contato')
% grid
% set(fig,'Units','Inches');
% pos = get(fig,'Position');
% set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
% if plots
%     print(fig,[Path,'\',Name,' - contact_forces'],'-dpng','-r0')
% end
% 
% fig = figure('Name',['Torque - ',Name],'DefaultAxesFontSize',12);
% hold on
% 	plot(DATA.time,DATA.tau_s,'-r','linewidth',2)
% 	plot(DATA.time,DATA.tau_e,'-b','linewidth',2)
% 	for i=2:numel(PhasesInt)-1
% 		plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
% 	end
% hold off
% legend('\tau_e','\tau_s','Location','Best')
% xlabel('tempo [s]'),ylabel('forces [N]'),title('Torques')
% grid
% set(fig,'Units','Inches');
% pos = get(fig,'Position');
% set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
% if plots
%     print(fig,[Path,'\',Name,' - torques'],'-dpng','-r0')
% end
%     
% if isfield(Results.solution.phase(1),'state')
% 	fig = figure('Name',['Velocity Comparison - ',Name],'DefaultAxesFontSize',12);
% 	hold on
% 		plot(DATA.time,DATA.dveli,'-b','linewidth',2)
% 		plot(DATA.time,DATA.dtheta*PAR.Rr,'-r','linewidth',2)
% 		for i=2:numel(PhasesInt)-1
% 			plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
% 		end
% 	hold off
% 	legend(['v_{imp}',char(10),sprintf('M_i = %0.2f',Results.Options.ImpedanceMass),char(10),sprintf('C_i = %0.2f',Results.Options.ImpedanceFriction)],'v_{real}','Location','Best')
% 	xlabel('tempo [s]'),ylabel('velocity [m/s]'),title('Velocidades')
% 	grid
%     set(fig,'Units','Inches');
%     pos = get(fig,'Position');
%     set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
%         % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
%     % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
%     if plots
%         print(fig,[Path,'\',Name,' - velocity_comparison'],'-dpng','-r0')
%     end
%     
% 	fig = figure('Name',['Applied Torques - ',Name],'DefaultAxesFontSize',12);
% 	hold on
% 		plot(DATA.time,DATA.taup,'-r','linewidth',2)
% 		plot(DATA.time,PAR.rt*PAR.Kt*DATA.imotor,'-b','linewidth',2)
% 		for i=2:numel(PhasesInt)-1
% 			plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
% 		end
% 	hold off
% 	legend('\tau_p','\tau_m','Location','Best')
% 	xlabel('tempo [s]'),ylabel('Torques [Nm]'),title('Torques Aplicados')
% 	grid
%     set(fig,'Units','Inches');
%     pos = get(fig,'Position');
%     set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
%     % C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
%     % C:\Users\Vinicius\Google Drive\Tese\Tex\figures
%     if plots
%         print(fig,[Path,'\',Name,' - applied_torques'],'-dpng','-r0')
%     end
% end
%
%% Data Plot
if nargin > 3
    % Data Analysis (Reference)
%     DATA = fvalid(PAR.Reference, Results.Reference);
    fileID = fopen([dPath,'\',Name,'_Rfrc0.txt'],'w');
    fprintf(fileID,'%6s %12s %12s\r\n','time','vlct','tpers');
    fprintf(fileID,'%6.2f %12.8f %12.8f\r\n',[DATA.Reference.Results.time';DATA.Reference.Results.dtheta'*DATA.Reference.Person.Rr;DATA.Reference.Results.taup'/DATA.Reference.Person.R]);
    fclose(fileID);
    
    % Data Analysis (Controller)
    for q=1:numel(DATA.PIController)
%         DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
        fileID = fopen([dPath,'\',Name,'_Cntl',num2str(q),'.txt'],'w');
        fprintf(fileID,'%6s %12s %12s %12s\r\n','time','vlct','tpers','tmotr');
        Kmotor = DATA.PIController{q}.Person.rt*DATA.PIController{q}.Person.Kt;
        fprintf(fileID,'%6.2f %12.8f %12.8f %12.8f\r\n',[DATA.PIController{q}.Results.time';DATA.PIController{q}.Results.dtheta'*DATA.PIController{q}.Person.Rr;DATA.PIController{q}.Results.taup'/DATA.PIController{q}.Person.R;Kmotor*DATA.PIController{q}.Results.imotor'/DATA.PIController{q}.Person.Rr]);
        fclose(fileID);
    end
end


%% Hand Pattern

PhasesInt = 1;
for i=1:numel(DATA.Reference.Raw.solution.phase)
	if i==1
		PhasesInt = [PhasesInt, numel(DATA.Reference.Raw.solution.phase(i).time)];
	else
		PhasesInt = [PhasesInt, numel(DATA.Reference.Raw.solution.phase(i).time)-1+PhasesInt(end)];
	end
end

xi = -DATA.Reference.Person.h+DATA.Reference.Person.A*cos(DATA.Reference.Results.alpha)+DATA.Reference.Person.B*cos(DATA.Reference.Results.beta);
yi = DATA.Reference.Person.Y-DATA.Reference.Person.A*sin(DATA.Reference.Results.alpha)-DATA.Reference.Person.B*sin(DATA.Reference.Results.beta);

fig = figure('Name',['Perfil da mão - ',Name],'DefaultAxesFontSize',12);
rows = ceil((numel(DATA.PIController)+1)/2);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
subplot(rows,2,1)
set(gca, 'FontName', 'Times New Roman')
hold on
%     PhasesInt = 1;
%     for i=1:numel(Results.Reference.solution.phase)
%         if i==1
%             PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)];
%         else
%             PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)-1+PhasesInt(end)];
%         end
%     end

    % Data Analysis (Reference)
%     DATA = fvalid(PAR.Reference, Results.Reference);
%     
%     xi = -PAR.Reference.h+PAR.Reference.A*cos(DATA.alpha)+PAR.Reference.B*cos(DATA.beta);
%     yi = PAR.Reference.Y-PAR.Reference.A*sin(DATA.alpha)-PAR.Reference.B*sin(DATA.beta);
    
    th = linspace(-pi,0,20)';
%     th = linspace(PAR.Reference.thetac1,PAR.Reference.thetac2,20)';
    xunit2 = DATA.Reference.Person.R * cos(th);
    yunit2 = -DATA.Reference.Person.R * sin(th);
	for i=1:numel(PhasesInt)-1
		if rem(i,2)
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-b','linewidth',2.5)
		else
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-.r','linewidth',2.5)
		end
	end
	plot(xunit2,yunit2,'-m')
    plot([0;DATA.Reference.Person.R*cos(DATA.Reference.Person.thetac2)],[0;-DATA.Reference.Person.R*sin(DATA.Reference.Person.thetac2)],'.-m', ...
    [0;DATA.Reference.Person.R*cos(DATA.Reference.Person.thetac1)],[0;-DATA.Reference.Person.R*sin(DATA.Reference.Person.thetac1)],'.-m','linewidth',0.5)
hold off
axis('equal'),title('A) Reference')
set(gca, 'xtick',[])
set(gca, 'ytick',[])
set(gca, 'xticklabel',[])
set(gca, 'yticklabel',[])
grid, box on

for q=1:numel(DATA.PIController)
    subplot(rows,2,q+1)
    set(gca, 'FontName', 'Times New Roman')
    hold on
        PhasesInt = 1;
        for i=1:numel(DATA.PIController{q}.Raw.solution.phase)
            if i==1
                PhasesInt = [PhasesInt, numel(DATA.PIController{q}.Raw.solution.phase(i).time)];
            else
                PhasesInt = [PhasesInt, numel(DATA.PIController{q}.Raw.solution.phase(i).time)-1+PhasesInt(end)];
            end
        end

        % Data Analysis (Reference)
%         DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
    
        xi = -DATA.PIController{q}.Person.h+DATA.PIController{q}.Person.A*cos(DATA.PIController{q}.Results.alpha)+DATA.PIController{q}.Person.B*cos(DATA.PIController{q}.Results.beta);
        yi = DATA.PIController{q}.Person.Y-DATA.PIController{q}.Person.A*sin(DATA.PIController{q}.Results.alpha)-DATA.PIController{q}.Person.B*sin(DATA.PIController{q}.Results.beta);
    
        for i=1:numel(PhasesInt)-1
            if rem(i,2)
                plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-b','linewidth',2.5)
            else
                plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-.r','linewidth',2.5)
            end
        end
        plot(xunit2,yunit2,'-m')
        plot([0;DATA.PIController{q}.Person.R*cos(DATA.PIController{q}.Person.thetac2)],[0;-DATA.PIController{q}.Person.R*sin(DATA.PIController{q}.Person.thetac2)],'.-m', ...
        [0;DATA.PIController{q}.Person.R*cos(DATA.PIController{q}.Person.thetac1)],[0;-DATA.PIController{q}.Person.R*sin(DATA.PIController{q}.Person.thetac1)],'.-m','linewidth',0.5)
    hold off
    axis('equal'),title([char(q + 65),') M_{',num2str(SimuCases(q,1)*100),'}-C_{',num2str(SimuCases(q,2)*100),'}'])
    set(gca, 'xtick',[])
    set(gca, 'ytick',[])
    set(gca, 'xticklabel',[])
    set(gca, 'yticklabel',[])
    grid, box on
end
set(fig,'Units','Inches');
pos = get(fig,'Position');
set(fig,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures
if plots
    print(fig,[Path,'\',Name,'_-_HP'],Format,'-r0')
end