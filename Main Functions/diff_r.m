function [ dy ] = diff_r( t,y,type )
%
%    Diferenciação linear de y rearranjados em intervalos de tempo delta(t)
%    forma:
%            dy = diff_r(t,y,type)
%    sendo:
%           t = vetor de tempo;
%           y = variável a ser diferenciada;
%           dy = vetor diferencial de y;
%           type = tipo de diferenciação a ser usado, pode ser os seguintes
%           valores:
%                   FD - diferença frontal;
%                   BD - diferença traseira;
%                   CD - diferença central;
%                   GD - por Gauss (para muitos pontos);
%           Obs.: Se não dado o valor de 'type', será considerado
%           diferenciação frontal.
%
if nargin ==2
for i=1:numel(t)-1
    
    dy(i)=(y(i+1)-y(i))/(t(i+1)-t(i));
    
end

dy = [dy dy(numel(dy))]';
else
    if size(y,1)<size(y,2)
        Trans = '''';
    else
        Trans = '';
    end
    switch type
        case 'CD'
            y2 = interp1(t,y,linspace(0-1/(2*numel(y)),1+1/(2*numel(y)),numel(y)*2+2)*(t(end)-t(1))+t(1),'linear','extrap');
            t2 = linspace(0-1/(2*numel(y)),1+1/(2*numel(y)),numel(y)*2+2)*(t(end)-t(1))+t(1);
            step = diff(t2);
            dy2 = 1/(2*step(1))*(y2(3:end)-y2(1:end-2));
            dy = interp1(t2(2:end-1),dy2,t,'linear','extrap');
%             eval(['dy = dy',Trans,';']);
            
        case 'FD'
%             t2 = [t;1/numel(y)+t(end)];
            t2 = full(sparse(1:numel(y),1,t,numel(y)+1,1)+sparse(numel(y)+1,1,1/numel(y)+t(end),numel(y)+1,1));
            y2 = interp1(t,y,t2,'linear','extrap');
            dy = (y2(2:end)-y2(1:end-1))./(t2(2:end)-t2(1:end-1));
            eval(['dy = dy',Trans,';']);
            
        case 'BD'
            t2 = full(sparse(2:numel(y)+1,1,t,numel(y)+1,1)+sparse(1,1,1/numel(y)-t(1),numel(y)+1,1));
            y2 = interp1(t,y,t2,'linear','extrap');
            dy = (y2(2:end)-y2(1:end-1))./(t2(2:end)-t2(1:end-1));
            eval(['dy = dy',Trans,';']);
            
        case 'GD'
%             tau = (t-0.5*(t(end)+t(1)))*2/(t(end)-t(1));
            [tau,~] = legslb(2*numel(y));
            for j=1:numel(tau)
                for i=1:numel(tau)
                    D(i,j) = alternative_dl(j,tau,tau(i));
                end
            end
            t2 = 0.5*(t(end)-t(1))*tau+0.5*(t(end)+t(1));
            y2 = interp1(t,y,t2,'linear','extrap');
            dy2 = D*y2;
            dy = interp1(t2,dy2,t,'linear','extrap');
    end
end



end

