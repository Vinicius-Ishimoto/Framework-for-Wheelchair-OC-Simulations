function PARAM = wheelchair_parameters(varargin)

p = inputParser;

addOptional(p,'height',1.7,@isnumeric);
addOptional(p,'mass',70,@isnumeric);
addParameter(p,'angle',0,@isnumeric);
addParameter(p,'Rolling_resistive_force',20,@isnumeric);
addParameter(p,'thetai',100,@isnumeric);
addParameter(p,'Mi',-1,@isnumeric);
addParameter(p,'Ci',0,@isnumeric);
addParameter(p,'Fri',0,@isnumeric);
addParameter(p,'Mr',0,@isnumeric);
addParameter(p,'Cr',0,@isnumeric);
% addParameter(p,'isRef',false,@islogical);

parse(p,varargin{:});

height = p.Results.height;
mass = p.Results.mass;

%% Motores
% Bfric = 0.001; % coeficiente de viscosidade rotacional do motor [Ns]
% Kt = 0.0351;
% Ra = 2.875;
% La = 0.0085;
% Jm = 0.0083;
% Ke = 0.0351;
Bfric = 2*0.56;
Kt = 1.07;
Ra = 0.1/2;
La = 0.00244;
Jm = 2*0.04;
Ke = 1.07;
Kch = 1;
rt = 1; % relação de transmissão
delay = 0.08; % atraso da dinâmica de primeira ordem do torque

%% Parametros de impedancia
fm = 1; % fator de massa
fd = 1;  % fator de atrito dinâmico
Td = 0.1;
const = 50;
% PARAM.Mi = 100;
% PARAM.Ci = 15;

%% Optimal problem formulation
thetai = -p.Results.thetai*pi/180;
thetaf = -80*pi/180;
Frr = p.Results.Rolling_resistive_force;
Ang = p.Results.angle*pi/180; % Inclinação da pista

%% theta critico


%% Parametros padronizados
% if nargin == 0
%     height = 1.7;
%     mass   = 70;
% end
dFric = 0; % atrito dinâmico longitudinal [Ns/m]
% % Peso do indivíduo
% PARAM.mP = 69.500; % [kg] massa do corpo da pessoa
% PARAM.ma = 2*(0.022*PARAM.mP); % [kg] massa dos ante-braços + mãos
% % mA = 2*(0.016*mP); % [kg] massa dos ante-braços
% PARAM.mb = 2*(0.028*PARAM.mP); % [kg] massa dos braços
% PARAM.mC =(0.900*PARAM.mP); % [kg] massa do corpo (tronco + pernas + cabeça)
% PARAM.hPes = 1.690; % [m] altura do indivíduo
% PARAM.h = 0.0911;
% PARAM.Y = 0.6532;
% 
% PARAM.A = 0.3348;
% PARAM.B = 0.2282;
% PARAM.a = (0.682*PARAM.A); % [m] distância do CG do antebraço + mãos # cotovelo
% % a = (0.430*A); % [m] distância do CG do antebraço sem mãos # cotovelo
% PARAM.b = (0.436*PARAM.B); % [m] distância do CG do braço # ombro
% PARAM.Ja = 2*(0.022*PARAM.mP*(PARAM.A*0.468)^2); % [kg*m^2] momento de inércia dos dois antebraço + mãos
% % jA = 2*(0.016*mP*(A*0.468)^2); % [kg*m^2] momento de inércia dos dois antebraço sem mãos
% PARAM.Jb = 2*(0.028*PARAM.mP*(PARAM.B*0.322)^2); % [kg*m^2] momento de inércia dos dois braços
% %%%%%%%%%%%%%%%%%%%%%%%%%%%% Parâmetros da cadeira
% PARAM.mCd = 9.52; % [kg] massa do quadro da cadeira de rodas (12.000 Kg)
% PARAM.m_roda1 = 1.65; % [kg] massa da roda simples
% PARAM.m_roda2 = 4.35; % [kg] massa da roda instrumentada (smartwheel)
% PARAM.mr = PARAM.m_roda1 + PARAM.m_roda2; % [kg] massa das duas rodas da cadeira
% PARAM.R = 0.26625; % [m] raio do aro de propulsão (diâmetro de 22 polegadas = 0.5588 m)
% PARAM.Rr = 0.29875; % [m] raio da roda da cadeira
% PARAM.R3 = 0.02; % [m] raio do cubo da roda
% %jR = 2*(0.185806); % [kg*m^2] momento de inércia da roda
% PARAM.jR_roda1 = PARAM.m_roda1*((PARAM.R+PARAM.Rr)/2)^2; % [kg*m^2] momento de inércia da roda simples
% PARAM.jR_roda2 = PARAM.m_roda2*((PARAM.R+PARAM.Rr)/2)^2; % [kg*m^2] momento de inércia da roda instrumentada
% PARAM.Jr = PARAM.jR_roda1 + PARAM.jR_roda2; % [kg*m^2] momento de inércia das duas rodas
% PARAM.tp = 0;
% PARAM.mw = PARAM.mC + PARAM.mCd; % [kg] massa do corpo + cadeira
% PARAM.Jf = PARAM.Jr + PARAM.Rr^2*PARAM.ma + PARAM.Rr^2*PARAM.mb + PARAM.Rr^2*PARAM.mr + PARAM.Rr^2*PARAM.mw;
% PARAM.Mf = PARAM.Jf/PARAM.Rr^2;
% PARAM = repmat(struct(),numel(height),1);
for q=1:numel(height)
    aux = winter_parameters(height(q),mass(q));
    aux.dFric = dFric;
    aux.Bfric = Bfric;
    aux.delay = delay;
    aux.Kt = Kt;
    aux.Ra = Ra;
    aux.La = La;
    aux.Jm = Jm;
    aux.Ke = Ke;
    aux.const = const;
    aux.Kch = Kch;
    aux.rt = rt;
    aux.Jt = aux.Jf + 2*aux.Jm*aux.rt^2;
    aux.Mt = aux.Jt/aux.Rr^2;
    aux.Bf = aux.dFric*aux.Rr^2;
    aux.Bt = aux.Bf+2*aux.rt^2*aux.Bfric;
    if (p.Results.Mi >= 0)
        aux.Mi = p.Results.Mi;
        aux.Ci = p.Results.Ci;
        aux.Fri = p.Results.Fri;
    else
        aux.Mi = aux.Mf*fm;
        aux.Ci = aux.dFric*fd;
        aux.Fri = 0;
    end
    aux.Td = Td;
    aux.thetai = thetai;
    aux.thetaf = thetaf;
    de2 = acos((aux.R^2+aux.Y^2+aux.h^2-(aux.B+aux.A)^2)/(2*aux.R*sqrt(aux.Y^2+aux.h^2)));
    aux.thetac1=-(de2+pi/2-atan(-aux.h/aux.Y));
    if aux.thetac1<-pi
        aux.thetac1 = -pi;
    end
    aux.thetac2=-(pi/2-de2-atan(-aux.h/aux.Y));
    if aux.thetac2>0
        aux.thetac2 = 0;
    end
	MaxContact = aux.thetac2-aux.thetac1;
	aux.thetac1 = aux.thetac1 + MaxContact*0.05;
	aux.thetac2 = aux.thetac2 - MaxContact*0.05;
	if aux.thetac2*1.1 > aux.thetai
		aux.thetaf = aux.thetac2*1.1;
	else
		aux.thetaf = aux.thetac2;
    end
    %% Admitted
    aux.Frr = 20;
    aux.Ang = 0;
    
    M1 = four_bar_system([-pi/2,-pi/2,0],[0,0,0],aux,'Type','Reference');
    iM1 = inv(M1);
    aux.Jmt = 1/iM1(3,3);
    M1 = four_bar_system([-pi/2,-pi/2,0],[0,0,0],aux,'Type','Control');
    iM1 = inv(M1);
    aux.Jmf = 1/iM1(3,3);
    if (p.Results.Mr ~= 0)
        aux.Jmf = p.Results.Mr*aux.Rr^2 + aux.Jm*aux.rt^2;
        aux.Bt = p.Results.Cr*aux.Rr^2 + aux.rt^2*aux.Bfric;
    end
    %% External Influences
    aux.Frr = Frr;
    aux.Ang = Ang;
    PARAM(q) = aux;
end


function PAR = winter_parameters(height,mass)

PAR.ma = 2*(0.028*mass);       % [kg] massa do antebraço (1.420 kg)
PAR.mb = 2*(0.022*mass);       % [kg] massa do braço (1.820 kg)
PAR.mc = mass-PAR.ma-PAR.mb;   % [kg] massa do corpo sem braços e mãos
PAR.h = 0.05; % 0.0911;
PAR.Y = 0.75; %0.12+0.338*height;
PAR.A = 0.186*height;
PAR.B = (0.146+0.108/2)*height; % (0.146+0.108/2)*height;
PAR.a = (0.436*PAR.A);
PAR.b = (0.682*(0.146*height));
PAR.Ja = 1*(PAR.ma*(PAR.A*0.322)^2);
PAR.Jb = 1*(PAR.mb*((0.146*height)*0.468)^2);

PAR.mCd = 9.52;   % [kg] massa da cadeira sem as rodas.
PAR.mr = 2*1.65;  % [kg] massa das rodas
PAR.mw = PAR.mc + PAR.mCd;
PAR.R = 0.25695;  % [m] raio do aro de propulsão (diâmetro de 22 polegadas = 0.5588 m)
PAR.Rr = 0.29875; % [m] raio da roda da cadeira
PAR.Jr = 0.5*PAR.mr*((PAR.R+PAR.Rr)/2)^2;

PAR.Jf = PAR.Jr + PAR.Rr^2*(PAR.mCd + mass + PAR.mr);
PAR.Mf = PAR.Jf/PAR.Rr^2;