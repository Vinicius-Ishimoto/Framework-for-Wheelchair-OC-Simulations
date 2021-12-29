function [  T, ...
           TV, ...
           TA, ...
           TP ] = MTG_smooth(Tact_sf,Tact_se,Tact_ef,Tact_ee,theta_s,theta_e,dtheta_s,dtheta_e)       

    % Dados antropométricos na Tese do Colin Brown (Apêndice B).
    % Shoulder + no sentido anti-horário
    % Elbow + no sentido anti-horário
    % Ângulos relativos 
    %    Shoulder: braço em relação ao tronco
    %    Elbow: ante-braço em relação ao braço 
    %
    % Saidas
    %       T  = [T_sf,  T_se,  T_ef,  T_ee]
    %       TV = [TV_sf, TV_se, TV_ef, TV_ee]
    %       TA = [TA_sf, TA_se, TA_ef, TA_ee]
    %       TP = [TP_s, TP_e]
    %       T_sf = Torque shoulder flexion
    %       T_se = Torque shoulder extension
    %       T_ef = Torque elbow flexion
    %       T_ee = Torque elbow extension
    %       TP_s = Torque passive shoulder
    %       TP_e = Torque passive flexion
    % Entradas
    %   Tact_sf = Activation shoulder flexion   [0 ... 1/2 ... 1]
    %   Tact_se = Activation shoulder extension [0 ........... 1]
    %   Tact_ef = Activation elbow flexion      [0 ........... 1]
    %   Tact_ee = Activation elbow extension    [0 ... 1/2 ... 1]
    %   theta_s = Theta soulder                 [-30º ...... 60º]
    %   theta_e = Theta elbow                   [0º ....... 120º]
    %  dtheta_s = dot Theta soulder             [-2 .... 2 rad/s]
    %  dtheta_e = dot Theta elbow               [-2 .... 2 rad/s]
    
    factor = 0;

%% TVs
 
	TVs_sf = ifThenElse(dtheta_s >= 0, TV_sfp(dtheta_s), TV_sfn(dtheta_s), factor);
    % if dtheta_s >= 0
        % To = 80.3; wmax = 2000; wc = 750; 
        % Tc = To*wc/wmax;
        % C = Tc*(wmax + wc); 
        % TVs_sf = C/(wc + dtheta_s*180/pi) - Tc;
    % else
        % To = 80.3; wmax = 2000; wc = 750; k = 4.3;
        % Tm = 1.4*To;
        % we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        % E = -(Tm - To)*we;
        % TVs_sf = E/(we - dtheta_s*180/pi) + Tm;    % original - Tm
    % end
    
    temp_s = -dtheta_s; % ajuste de sinal para seguir a convenção adotada
	TVs_se = ifThenElse(temp_s >= 0, TV_sep(temp_s), TV_sen(temp_s), factor);
    % if temp_s >= 0
        % To = 110; wmax = 2000; wc = 438.53;
        % Tc = To*wc/wmax;
        % C = Tc*(wmax + wc); 
        % TVs_se = C/(wc + temp_s*180/pi) - Tc;
    % else
        % To = 110; wmax = 2000; wc = 438.53; k = 4.3;
        % Tm = 1.4*To;
        % we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        % E = -(Tm - To)*we;
        % TVs_se = E/(we - temp_s*180/pi) + Tm;
    % end
%
	TVs_ef = ifThenElse(dtheta_e >= 0, TV_efp(dtheta_e), TV_efn(dtheta_e), factor);
    % if dtheta_e >= 0
        % To = 74.8; wmax = 2000; wc = 750; 
        % Tc = To*wc/wmax;
        % C = Tc*(wmax + wc); 
        % TVs_ef = C/(wc + dtheta_e*180/pi) - Tc;
    % else
        % To = 74.8; wmax = 2000; wc = 750; k = 4.3;
        % Tm = 1.4*To;
        % we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        % E = -(Tm - To)*we;
        % TVs_ef = E/(we - dtheta_e*180/pi) + Tm;
    % end
    
    temp_e = -dtheta_e;  % ajuste de sinal para seguir a convenção adotada
	TVs_ee = ifThenElse(temp_e >= 0, TV_eep(temp_e), TV_een(temp_e), factor);
    % if temp_e >= 0
        % To = 52.8; wmax = 1999.99; wc = 614.17;
        % Tc = To*wc/wmax;
        % C = Tc*(wmax + wc); 
        % TVs_ee = C/(wc + temp_e*180/pi) - Tc;
    % else
        % To = 52.8; wmax = 1999.99; wc = 614.17; k = 4.3;
        % Tm = 1.4*To;
        % we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        % E = -(Tm - To)*we;
        % TVs_ee = E/(we - temp_e*180/pi) + Tm;
    % end                

%% a(w) 

    amax = 1;
%    
    amin = 0.66; wr = 90; w1 = 17.25;
    a_sf = amin + (amax - amin)/(1 + exp( -(dtheta_s*180/pi - w1)/wr) );
%     a_sf = 1;
%    
    amin = 0.77; wr = 90; w1 = 90;  
    a_se = amin + (amax - amin)/(1 + exp( -(dtheta_s*180/pi - w1)/wr) );
%     a_se = 1;
%
    amin = 0.99; wr = 90; w1 = -90;  
    a_ef = amin + (amax - amin)/(1 + exp( -(dtheta_e*180/pi - w1)/wr) );
%     a_ef = 1;
%    
    amin = 0.74; wr = 54.31; w1 = -41.94;
    a_ee = amin + (amax - amin)/(1 + exp( -(dtheta_e*180/pi - w1)/wr) );
%     a_ee = 1;
    
%% TV

    TV_sf = (TVs_sf)*(a_sf);  %
    TV_se = (TVs_se)*(a_se);
    TV_ef = (TVs_ef)*(a_ef);
    TV_ee = (TVs_ee)*(a_ee); 
    
    TV = [TV_sf, TV_se, TV_ef, TV_ee];

%% TA (ok)
    
    a = -0.109; b = 0.180; c = 0.144; d = -0.287; e = +0.817;  % original e = -0.817
    TA_sf = a*theta_s^4 + b*theta_s^3 + c*theta_s^2 + d*theta_s^1 + e;
%    
    a = -0.230; b = 0.389; c = 0.794;    % original fala que é de 4a ordem (apêndice)
    TA_se = a*theta_s^2 + b*theta_s^1 + c;
%
    a = -0.279; b = 0.576; c = 0.580;    
    TA_ef = a*theta_e^2 + b*theta_e^1 + c;
%    
    a = -0.270; b = 0.588; c = 0.636;    
    TA_ee = a*theta_e^2 + b*theta_e^1 + c;
    

%% TP
                                                             % original +k4 na fórmula  
    O1 = -1.14;  O2 = 1.27; k1 = 7.03; k2 = 2.31; k3 = 4.30; k4 = 1.65; C = 0.1;
    TP_s = k1*exp(-k2*(theta_s - O1)) - k3*exp(-k4*(O2 - theta_s)) - C*(dtheta_s);
%
    O1 = -0.198; O2 = 2.79; k1 = 1.78; k2 = 1.73; k3 = 1.42; k4 = 2.46; C = 0.1;
    TP_e = k1*exp(-k2*(theta_e - O1)) - k3*exp(-k4*(O2 - theta_e)) - C*(dtheta_e);
    
    TP = [TP_s, TP_e];

%% T

    TVA_sf = +(Tact_sf)*(TV_sf*TA_sf);  % sinal ajustado conforme convenção
    TVA_se = -(Tact_se)*(TV_se*TA_se);  % sinal ajustado conforme convenção
    TVA_ef = +(Tact_ef)*(TV_ef*TA_ef);  % sinal ajustado conforme convenção
    TVA_ee = -(Tact_ee)*(TV_ee*TA_ee);  % sinal ajustado conforme convenção
    T_sf = TVA_sf + TP_s;
    T_se = TVA_se + TP_s;
    T_ef = TVA_ef + TP_e;
    T_ee = TVA_ee + TP_e;
    
    T = [T_sf, T_se, T_ef, T_ee];
    TA = [TVA_sf, TVA_se, TVA_ef, TVA_ee];

end

function out = TV_sfp(dtheta_s)

	To = 80.3; wmax = 2000; wc = 750; 
	Tc = To*wc/wmax;
	C = Tc*(wmax + wc); 
	out = C/(wc + dtheta_s*180/pi) - Tc;

end

function out = TV_sfn(dtheta_s)

	To = 80.3; wmax = 2000; wc = 750; k = 4.3;
	Tm = 1.4*To;
	we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
	E = -(Tm - To)*we;
	out = E/(we - dtheta_s*180/pi) + Tm;    % original - Tm

end

function out = TV_sep(dtheta_s)

	To = 110; wmax = 2000; wc = 438.53;
	Tc = To*wc/wmax;
	C = Tc*(wmax + wc); 
	out = C/(wc + dtheta_s*180/pi) - Tc;

end

function out = TV_sen(dtheta_s)

To = 110; wmax = 2000; wc = 438.53; k = 4.3;

    Tm = 1.4*To;
    we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
    E = -(Tm - To)*we;
    out = E/(we - dtheta_s*180/pi) + Tm;

end

function out = TV_efp(dtheta_e)

	To = 74.8; wmax = 2000; wc = 750; 
    Tc = To*wc/wmax;
    C = Tc*(wmax + wc); 
    out = C/(wc + dtheta_e*180/pi) - Tc;

end

function out = TV_efn(dtheta_e)

	To = 74.8; wmax = 2000; wc = 750; k = 4.3;
    Tm = 1.4*To;
    we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
    E = -(Tm - To)*we;
    out = E/(we - dtheta_e*180/pi) + Tm;

end

function out = TV_eep(dtheta_e)

	To = 52.8; wmax = 1999.99; wc = 614.17;
    Tc = To*wc/wmax;
    C = Tc*(wmax + wc); 
    out = C/(wc + dtheta_e*180/pi) - Tc;

end

function out = TV_een(dtheta_e)

	To = 52.8; wmax = 1999.99; wc = 614.17; k = 4.3;
    Tm = 1.4*To;
    we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
    E = -(Tm - To)*we;
    out = E/(we - dtheta_e*180/pi) + Tm;

end

