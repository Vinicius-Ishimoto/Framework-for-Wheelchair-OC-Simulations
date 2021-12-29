function fgraphicW(PAR,rt,varargin)

p = inputParser;

addParameter(p,'Density',1,@isnumeric);
addParameter(p,'Movement',false,@islogical);
addParameter(p,'Path','',@ischar);
addParameter(p,'Name','',@ischar);
addParameter(p,'Format','pdf',@ischar);
% addParameter(p,'isRef',false,@islogical);
parse(p,varargin{:});

if length([p.Results.Path,p.Results.Name]) < 3
    plots = false;
    p.Results.Name = 'Stick Figure';
else
    plots = true;
end


vector = linspace(1,numel(rt.theta),floor(numel(rt.theta)*p.Results.Density));
vector = floor(vector);
x0 = 0;

fig = figure('Name',p.Results.Name,'DefaultAxesFontSize',12);
set(fig,'PaperPositionMode','Auto','PaperUnits','Centimeters','Units','Centimeters','PaperSize',[17, 12],'Position',[0 0 17 12])
hold on
if not(p.Results.Movement)
    th = linspace(0,2*pi,35)';
    xunit2 = PAR.h+ PAR.R * cos(th);
    yunit2 = -PAR.Y-PAR.R * sin(th);
    plot(xunit2,yunit2,'-k','linewidth',1)
%     plot(xunit2,yunit2,'-k','Marker','v','linewidth',2)
end
for i=vector
    if p.Results.Movement
        x0 = rt.x(i);
    end
    ax = [x0,x0+PAR.A*cos(rt.alpha(i))];
    ay = -[0,PAR.A*sin(rt.alpha(i))];
    bx = [ax(2),ax(2)+PAR.B*cos(rt.beta(i))];
    by = [ay(2),ay(2)-PAR.B*sin(rt.beta(i))];
    cx = [PAR.h+x0,PAR.h+x0+PAR.R*cos(rt.theta(i))];
    cy = [-PAR.Y,-PAR.Y-PAR.R*sin(rt.theta(i))];
    plot(ax,ay,'-r',bx,by,'-g')
%     plot(ax,ay,'-r',bx,by,'-g',cx,cy,'-k')
    if not(p.Results.Movement)
        axis('equal')
    end
    set(gca,'xtick',[])
    set(gca,'ytick',[])
    pause(0.05)
end

set(fig,'PaperPositionMode','Auto')
box off
axis off
% C:\Users\Vinicius\Google Drive\Tese\Disserta磯\Simula絥s\v1.1
% C:\Users\Vinicius\Google Drive\Tese\Tex\figures\results
if plots
    switch p.Results.Format
        case 'pdf'
            print(fig,[p.Results.Path,'\',p.Results.Name],'-dpdf','-r0')
        case 'eps'
            print(fig,[p.Results.Path,'\',p.Results.Name],'-depsc','-r0')
    end
end

end