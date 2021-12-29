function plot_graphics_combined(PAR,Results,Name,Path,dPath)

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

Mrk = ['o';'+';'x';'s';'d';'^'];

iClr = 1;

Lgd = {'Reference'};

%% Phases interfaces
PhasesInt = 1;
for i=1:numel(Results.Reference.solution.phase)
	if i==1
		PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)];
	else
		PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)-1+PhasesInt(end)];
	end
end

%% Graphic

fig = figure('Name','Velocities','DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
set(gca, 'FontName', 'Times New Roman')
hold on
    % Data Analysis (Reference)
    DATA = fvalid(PAR.Reference, Results.Reference);
%     Mki = ceil(linspace(1,length(DATA.time),20));
%     h(1,1) = plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Reference.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%     set(h(1,1),'Visible','off')
    plot(DATA.time,DATA.dtheta*PAR.Reference.Rr,'Color',Clr(iClr,:),'linewidth',2)
%     plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Reference.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(Results.Controller)
        DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
%         Mki = ceil(linspace((1+q),length(DATA.time),20));
%         h(1+q,1) = plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%         set(h(1+q,1),'Visible','off')
        plot(DATA.time,DATA.dtheta*PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'linewidth',2)
%         plot(DATA.time(Mki),DATA.dtheta(Mki)*PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
        Lgd{iClr} = {['Case ',num2str(iClr-1)]};
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
    print(fig,[Path,'\',Name,'_-_Velocity'],'-dpdf','-r0')
end


clear h
fig = figure('Name','Forces','DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
set(gca, 'FontName', 'Times New Roman')
iClr = 1;
hold on
    % Data Analysis (Reference)
    DATA = fvalid(PAR.Reference, Results.Reference);
%     Mki = ceil(linspace(1,length(DATA.time),40));
%     h(1,1) = plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Reference.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%     set(h(1,1),'Visible','off')
    plot(DATA.time,DATA.taup/PAR.Reference.R,'Color',Clr(iClr,:),'linewidth',2)
%     plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Reference.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
    iClr = iClr + 1;
    
    % Data Analysis (Controller)
    for q=1:numel(Results.Controller)
        DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
%         Mki = ceil(linspace((1+q),length(DATA.time),40));
%         h(1+q,1) = plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Controller{q}.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%         set(h(1+q,1),'Visible','off')
        plot(DATA.time,DATA.taup/PAR.Controller{q}.R,'Color',Clr(iClr,:),'linewidth',2)
%         plot(DATA.time(Mki),DATA.taup(Mki)/PAR.Controller{q}.R,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
        Lgd{iClr} = {['Case ',num2str(iClr-1)]};
        iClr = iClr + 1;
    end
    
%     % Phases Interface
%     for i=2:numel(PhasesInt)-1
%         plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
%     end
hold off
xlabel('Time [s]'),ylabel('Force�[N]'),title('User''s Tangencial Force')
% legend(h,[Lgd{:}],'Location','Best')
legend([Lgd{:}],'Location','Best')
grid, box on
set(fig,'PaperPositionMode','Auto')
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s\v1.1
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures\results
if plots
    print(fig,[Path,'\',Name,'_-_TForce'],'-dpdf','-r0')
end

clear h
fig = figure('Name','Forces','DefaultAxesFontSize',12);
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
    for q=1:numel(Results.Controller)
        DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
        Kmotor = PAR.Controller{q}.rt*PAR.Controller{q}.Kt;
%         Mki = ceil(linspace((1+q),length(DATA.time),30));
%         h(q,1) = plot(DATA.time(Mki),Kmotor*DATA.imotor(Mki)/PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:));
%         set(h(q,1),'Visible','off')
        plot(DATA.time,Kmotor*DATA.imotor/PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'linewidth',2)
%         plot(DATA.time(Mki),Kmotor*DATA.imotor(Mki)/PAR.Controller{q}.Rr,'Color',Clr(iClr,:),'Marker',Mrk(iClr,:),'LineStyle','none','MarkerSize',8);
        Lgd{iClr-1} = {['Case ',num2str(iClr-1)]};
        iClr = iClr + 1;
    end
    
%     % Phases
%     Interface
%     for i=2:numel(PhasesInt)-1
%         plot(DATA.time(PhasesInt(i))*[1,1],get(gca,'ylim'),'.-k','linewidth',1.5)
%     end
hold off
xlabel('Time [s]'),ylabel('Force�[N]'),title('Motor''s Tangencial Force')
% legend(h,[Lgd{:}],'Location','Best')
legend([Lgd{:}],'Location','Best')
grid, box on
set(fig,'PaperPositionMode','Auto')
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s\v1.1
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures\results
if plots
    print(fig,[Path,'\',Name,'_-_MotorForce'],'-dpdf','-r0')
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
if nargin > 4
    % Data Analysis (Reference)
    DATA = fvalid(PAR.Reference, Results.Reference);
    fileID = fopen([dPath,'\',Name,'_Rfrc0.txt'],'w');
    fprintf(fileID,'%6s %12s %12s\r\n','time','vlct','tpers');
    fprintf(fileID,'%6.2f %12.8f %12.8f\r\n',[DATA.time';DATA.dtheta'*PAR.Reference.Rr;DATA.taup'/PAR.Reference.R]);
    fclose(fileID);
    
    % Data Analysis (Controller)
    for q=1:numel(Results.Controller)
        DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
        fileID = fopen([dPath,'\',Name,'_Cntl',num2str(q),'.txt'],'w');
        fprintf(fileID,'%6s %12s %12s %12s\r\n','time','vlct','tpers','tmotr');
        Kmotor = PAR.Controller{q}.rt*PAR.Controller{q}.Kt;
        fprintf(fileID,'%6.2f %12.8f %12.8f %12.8f\r\n',[DATA.time';DATA.dtheta'*PAR.Controller{q}.Rr;DATA.taup'/PAR.Controller{q}.R;Kmotor*DATA.imotor'/PAR.Controller{q}.Rr]);
        fclose(fileID);
    end
end


%% Hand Pattern

PhasesInt = 1;
for i=1:numel(Results.Reference.solution.phase)
	if i==1
		PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)];
	else
		PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)-1+PhasesInt(end)];
	end
end

xi = -PAR.Reference.h+PAR.Reference.A*cos(DATA.alpha)+PAR.Reference.B*cos(DATA.beta);
yi = PAR.Reference.Y-PAR.Reference.A*sin(DATA.alpha)-PAR.Reference.B*sin(DATA.beta);

fig = figure('Name',['Perfil da mão - ',Name],'DefaultAxesFontSize',12);
rows = ceil((numel(Results.Controller)+1)/2);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
subplot(rows,2,1)
set(gca, 'FontName', 'Times New Roman')
hold on
    PhasesInt = 1;
    for i=1:numel(Results.Reference.solution.phase)
        if i==1
            PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)];
        else
            PhasesInt = [PhasesInt, numel(Results.Reference.solution.phase(i).time)-1+PhasesInt(end)];
        end
    end

    % Data Analysis (Reference)
    DATA = fvalid(PAR.Reference, Results.Reference);
    
    xi = -PAR.Reference.h+PAR.Reference.A*cos(DATA.alpha)+PAR.Reference.B*cos(DATA.beta);
    yi = PAR.Reference.Y-PAR.Reference.A*sin(DATA.alpha)-PAR.Reference.B*sin(DATA.beta);
    
    th = linspace(-pi,0,20)';
%     th = linspace(PAR.Reference.thetac1,PAR.Reference.thetac2,20)';
    xunit2 = PAR.Reference.R * cos(th);
    yunit2 = -PAR.Reference.R * sin(th);
	for i=1:numel(PhasesInt)-1
		if rem(i,2)
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-b','linewidth',2.5)
		else
			plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-.r','linewidth',2.5)
		end
	end
	plot(xunit2,yunit2,'-m')
    plot([0;PAR.Reference.R*cos(PAR.Reference.thetac2)],[0;-PAR.Reference.R*sin(PAR.Reference.thetac2)],'.-m', ...
    [0;PAR.Reference.R*cos(PAR.Reference.thetac1)],[0;-PAR.Reference.R*sin(PAR.Reference.thetac1)],'.-m','linewidth',0.5)
hold off
axis('equal'),title('Reference')
set(gca, 'xtick',[])
set(gca, 'ytick',[])
set(gca, 'xticklabel',[])
set(gca, 'yticklabel',[])
grid, box on

for q=1:numel(Results.Controller)
    subplot(rows,2,q+1)
    set(gca, 'FontName', 'Times New Roman')
    hold on
        PhasesInt = 1;
        for i=1:numel(Results.Controller{q}.solution.phase)
            if i==1
                PhasesInt = [PhasesInt, numel(Results.Controller{q}.solution.phase(i).time)];
            else
                PhasesInt = [PhasesInt, numel(Results.Controller{q}.solution.phase(i).time)-1+PhasesInt(end)];
            end
        end

        % Data Analysis (Reference)
        DATA = fvalid(PAR.Controller{q}, Results.Controller{q});
    
        xi = -PAR.Controller{q}.h+PAR.Controller{q}.A*cos(DATA.alpha)+PAR.Controller{q}.B*cos(DATA.beta);
        yi = PAR.Controller{q}.Y-PAR.Controller{q}.A*sin(DATA.alpha)-PAR.Controller{q}.B*sin(DATA.beta);
    
        for i=1:numel(PhasesInt)-1
            if rem(i,2)
                plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-b','linewidth',2.5)
            else
                plot(xi(PhasesInt(i):PhasesInt(i+1)),yi(PhasesInt(i):PhasesInt(i+1)),'-.r','linewidth',2.5)
            end
        end
        plot(xunit2,yunit2,'-m')
        plot([0;PAR.Controller{q}.R*cos(PAR.Controller{q}.thetac2)],[0;-PAR.Controller{q}.R*sin(PAR.Controller{q}.thetac2)],'.-m', ...
        [0;PAR.Controller{q}.R*cos(PAR.Controller{q}.thetac1)],[0;-PAR.Controller{q}.R*sin(PAR.Controller{q}.thetac1)],'.-m','linewidth',0.5)
    hold off
    axis('equal'),title(['Case ',num2str(q)])
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
    print(fig,[Path,'\',Name,'_-_HP'],'-dpdf','-r0')
end