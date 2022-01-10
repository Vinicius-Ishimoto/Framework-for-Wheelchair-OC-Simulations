function [  T, ...
           TV, ...
           TA, ...
           TP ] = muscle_torque_generators(Tact_sf,Tact_se,Tact_ef,Tact_ee,theta_s,theta_e,dtheta_s,dtheta_e)       

    % Antropometric data based on Colin Brown thesis (Apendix B).
    % Shoulder + is counter-clockwise
    % Elbow + is counter-clockwise
    % Angles reference
    %    Shoulder: upperarm in relation to the trunk
    %    Elbow: forearm in relation to the upper-arm
    %
    % Outputs
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
    % Inputs
    %   Tact_sf = Activation shoulder flexion   [0 ... 1/2 ... 1]
    %   Tact_se = Activation shoulder extension [0 ........... 1]
    %   Tact_ef = Activation elbow flexion      [0 ........... 1]
    %   Tact_ee = Activation elbow extension    [0 ... 1/2 ... 1]
    %   theta_s = Theta shoulder                [-70º ...... 60º]
    %   theta_e = Theta elbow                   [0º ....... 120º]
    %  dtheta_s = dot Theta shoulder            [-2 .... 2 rad/s]
    %  dtheta_e = dot Theta elbow               [-2 .... 2 rad/s]

%% TVs
 
    if dtheta_s >= 0
        To = 80.3; wmax = 2000; wc = 750; 
        Tc = To*wc/wmax;
        C = Tc*(wmax + wc); 
        TVs_sf = C/(wc + dtheta_s*180/pi) - Tc;
    else
        To = 80.3; wmax = 2000; wc = 750; k = 4.3;
        Tm = 1.4*To;
        we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        E = -(Tm - To)*we;
        TVs_sf = E/(we - dtheta_s*180/pi) + Tm;    % original reference: - Tm
    end
    
    temp_s = -dtheta_s; % sign follows the reference adopted
    if temp_s >= 0
        To = 110; wmax = 2000; wc = 438.53;
        Tc = To*wc/wmax;
        C = Tc*(wmax + wc); 
        TVs_se = C/(wc + temp_s*180/pi) - Tc;
    else
        To = 110; wmax = 2000; wc = 438.53; k = 4.3;
        Tm = 1.4*To;
        we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        E = -(Tm - To)*we;
        TVs_se = E/(we - temp_s*180/pi) + Tm;
    end
%
    if dtheta_e >= 0
        To = 74.8; wmax = 2000; wc = 750; 
        Tc = To*wc/wmax;
        C = Tc*(wmax + wc); 
        TVs_ef = C/(wc + dtheta_e*180/pi) - Tc;
    else
        To = 74.8; wmax = 2000; wc = 750; k = 4.3;
        Tm = 1.4*To;
        we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        E = -(Tm - To)*we;
        TVs_ef = E/(we - dtheta_e*180/pi) + Tm;
    end
    
    temp_e = -dtheta_e;  % sign follows the reference adopted
    if temp_e >= 0
        To = 52.8; wmax = 1999.99; wc = 614.17;
        Tc = To*wc/wmax;
        C = Tc*(wmax + wc); 
        TVs_ee = C/(wc + temp_e*180/pi) - Tc;
    else
        To = 52.8; wmax = 1999.99; wc = 614.17; k = 4.3;
        Tm = 1.4*To;
        we = (Tm - To)*wmax*wc/(k*To*(wmax + wc));
        E = -(Tm - To)*we;
        TVs_ee = E/(we - temp_e*180/pi) + Tm;
    end                

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
    
    a = -0.109; b = 0.180; c = 0.144; d = -0.287; e = +0.817;  % original: e = -0.817 (adapted for the reference)
    TA_sf = a*theta_s^4 + b*theta_s^3 + c*theta_s^2 + d*theta_s^1 + e;
%    
    a = -0.230; b = 0.389; c = 0.794;    % original is a fourth ordem (appendix)
    TA_se = a*theta_s^2 + b*theta_s^1 + c;
%
    a = -0.279; b = 0.576; c = 0.580;    
    TA_ef = a*theta_e^2 + b*theta_e^1 + c;
%    
    a = -0.270; b = 0.588; c = 0.636;    
    TA_ee = a*theta_e^2 + b*theta_e^1 + c;
    
    TA = [TA_sf, TA_se, TA_ef, TA_ee];

%% TP
                                                             % original: +k4 (adapted for the reference)
    O1 = -1.14;  O2 = 1.27; k1 = 7.03; k2 = 2.31; k3 = 4.30; k4 = 1.65; C = 0.1;
    TP_s = k1*exp(-k2*(theta_s - O1)) - k3*exp(-k4*(O2 - theta_s)) - C*(dtheta_s);
%
    O1 = -0.198; O2 = 2.79; k1 = 1.78; k2 = 1.73; k3 = 1.42; k4 = 2.46; C = 0.1;
    TP_e = k1*exp(-k2*(theta_e - O1)) - k3*exp(-k4*(O2 - theta_e)) - C*(dtheta_e);
    
    TP = [TP_s, TP_e];

%% T

    T_sf = +(Tact_sf)*(TV_sf*TA_sf) + TP_s;  % sign follows the reference adopted
    T_se = -(Tact_se)*(TV_se*TA_se) + TP_s;  % sign follows the reference adopted
    T_ef = +(Tact_ef)*(TV_ef*TA_ef) + TP_e;  % sign follows the reference adopted
    T_ee = -(Tact_ee)*(TV_ee*TA_ee) + TP_e;  % sign follows the reference adopted

end

