function out = fvalid(PAR,r)

% if not(isfield(r.solution.phase(1),'state'))
    if strcmp(r.Options.Type,'Reference')
	out = open_system(r,PAR);
    elseif strcmp(r.Options.Type,'MSC-Reference')
        out = open_MTG(r,PAR);
    elseif strcmp(r.Options.Type,'MSD-Reference')
        out = open_MTG_dyn(r,PAR);
    elseif strcmp(r.Options.Type,'MSD-Reference-Test')
        out = open_MTG_dyn_test(r,PAR);
    elseif strcmp(r.Options.Type,'MSC-PI_Controller')
        out = closed_MTG(r,PAR);
    elseif strcmp(r.Options.Type,'MSD-PI_Controller')
        out = closed_MTG_dyn(r,PAR);
% else
% 	if size(r.solution.phase(2).control,2)>2 
    elseif strcmp(r.Options.Type,'PID')
            out = closed_PID_Observer(r,PAR);
        elseif strcmp(r.Options.Type,'MBF')
            out = closed_MBF_Observer(r,PAR);
        elseif strcmp(r.Options.Type,'PIDSAT')
            out = closed_PIDSAT_Observer(r,PAR);
        elseif strcmp(r.Options.Type,'PI')
            out = closed_PI_Observer(r,PAR);
%         end
% 	else
% 		out = closed_PID_FirstOrder(r,PAR);
% 	end
end

% nsim = numel(r.solution.phase);
% theta = r.solution.phase(1).position(1);
% dtheta = r.solution.phase(1).velocity(1);
% time = r.solution.phase(1).time(1);
% alpha = r.solution.phase(1).control(1,3);
% beta = r.solution.phase(1).control(1,4);
% tau1 = r.solution.phase(1).control(1,1);
% tau2 = r.solution.phase(1).control(1,2);
% dalpha = 0;
% dbeta = 0;
% dveli = 0;
% ierr = 0;
% imotor = 0;
% if isfield(r.solution.phase(1),'state')
    % if size(r.solution.phase(1).state,2)==2
        % dveli = r.solution.phase(1).state(1,1);
        % ierr = r.solution.phase(1).state(1,2);
    % else
        % dveli = r.solution.phase(1).state(1,1);
        % ierr = r.solution.phase(1).state(1,2);
        % imotor = r.solution.phase(1).state(1,3);
        % derr = r.solution.phase(1).state(1,4);
    % end
% taup = r.solution.phase(1).control(1,5);
% else
    % taup = r.solution.phase(1).taup(1);
% end
% for q=1:nsim
    
    % if rem(q,2)
        % theta = [theta;r.solution.phase(q).position(2:end)];
        % dtheta = [dtheta;r.solution.phase(q).velocity(2:end)];
        % time = [time;r.solution.phase(q).time(2:end)];
        % alpha = [alpha;r.solution.phase(q).control(2:end,3)];
        % beta = [beta;r.solution.phase(q).control(2:end,4)];
        % tau1 = [tau1;r.solution.phase(q).control(2:end,1)];
        % tau2 = [tau2;r.solution.phase(q).control(2:end,2)];
        
        % [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end)),vect(r.solution.phase(q).velocity(2:end)),PAR);
        % dalpha = [dalpha;double(dalpha1)];
        % dbeta = [dbeta;double(dbeta1)];
        % if isfield(r.solution.phase(q),'state')
            % if size(r.solution.phase(q).state,2)==2
                % dveli = [dveli;r.solution.phase(q).state(2:end,1)];
                % ierr = [ierr;r.solution.phase(q).state(2:end,2)];
                % taup = [taup;r.solution.phase(q).control(2:end,5)];
            % else
                % dveli = [dveli;r.solution.phase(q).state(2:end,1)];
                % ierr = [ierr;r.solution.phase(q).state(2:end,2)];
                % imotor = [imotor;r.solution.phase(q).state(2:end,3)];
                % derr = [derr;r.solution.phase(q).state(2:end,4)];
                % taup = [taup;r.solution.phase(q).control(2:end,5)];
            % end
        % else
            % taup = [taup;r.solution.phase(q).taup(2:end)];
        % end
        
    % else
        % theta = [theta;r.solution.phase(q).position(2:end,3)];
        % dtheta = [dtheta;r.solution.phase(q).velocity(2:end,3)];
        % time = [time;r.solution.phase(q).time(2:end)];
        % alpha = [alpha;r.solution.phase(q).position(2:end,1)];
        % beta = [beta;r.solution.phase(q).position(2:end,2)];
        % tau1 = [tau1;r.solution.phase(q).control(2:end,1)];
        % tau2 = [tau2;r.solution.phase(q).control(2:end,2)];
        
        % dalpha = [dalpha;r.solution.phase(q).velocity(2:end,1)];
        % dbeta = [dbeta;r.solution.phase(q).velocity(2:end,2)];
        % if isfield(r.solution.phase(q),'state')
            % if size(r.solution.phase(q).state,2)==2
                % dveli = [dveli;r.solution.phase(q).state(2:end,1)];
                % ierr = [ierr;r.solution.phase(q).state(2:end,2)];
                % taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
            % else
                % dveli = [dveli;r.solution.phase(q).state(2:end,1)];
                % ierr = [ierr;r.solution.phase(q).state(2:end,2)];
                % imotor = [imotor;r.solution.phase(q).state(2:end,3)];
                % if size(r.solution.phase(q).control,2)>2
                    % taup = [taup;r.solution.phase(q).control(2:end,3)];
                % else
                    % taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
                % end
            % end
        % else
            % taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
        % end
    % end
% end

% out.time = time;
% out.alpha = alpha;
% out.beta = beta;
% out.theta = theta;
% out.dalpha = dalpha;
% out.dbeta = dbeta;
% out.dtheta = dtheta;
% out.taup = taup;
% out.tau1 = tau1;
% out.tau2 = tau2;
% if isfield(r.solution.phase(1),'state')
    % if size(r.solution.phase(1).state,2)==2
        % out.vmotor = dveli;
        % out.imotor = ierr;
    % else
        % out.dveli = dveli;
        % out.ierr = ierr;
        % out.imotor = imotor;
    % end
% end

function out = open_system(r,PAR)

nsim = numel(r.solution.phase);
nsize = 0;
for q=1:nsim
    nsize = nsize + numel(r.solution.phase(q).position(:,1)) - 1;
end
nsize = nsize+1;
theta = r.solution.phase(1).position(1);
dtheta = r.solution.phase(1).velocity(1);
time = r.solution.phase(1).time(1);
alpha = r.solution.phase(1).control(1,3);
beta = r.solution.phase(1).control(1,4);
tau1 = r.solution.phase(1).control(1,1);
tau2 = r.solution.phase(1).control(1,2);
tau3 = r.solution.phase(1).control(1,5);
tau4 = r.solution.phase(1).control(1,6);
ind = r.Options.Indice;
obj = r.solution.objective;
dalpha = 0;
dbeta = 0;
dveli = 0;
ierr = 0;
imotor = 0;
Fx3 = 0;
Fy3 = 0;
x = 0;

taup = r.solution.phase(1).taup(1);

for q=1:nsim
    
    if rem(q,2)
        theta = [theta;r.solution.phase(q).position(2:end,1)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end,1)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).control(2:end,3)];
        beta = [beta;r.solution.phase(q).control(2:end,4)];
        tau1 = [tau1;r.solution.phase(q).control(2:end,1)];
        tau2 = [tau2;r.solution.phase(q).control(2:end,2)];
        tau3 = [tau3;r.solution.phase(q).control(2:end,5)];
        tau4 = [tau4;r.solution.phase(q).control(2:end,6)];
        x = [x;(r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(end)];
		if q == 1
			obj = [obj;trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,1).^ind+r.solution.phase(q).control(:,2).^ind+r.solution.phase(q).control(:,5).^ind+r.solution.phase(q).control(:,6).^ind)];
            
        else
			obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,5).^ind+r.solution.phase(q).control(2:end,6).^ind)];
        end
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end,1)),vect(r.solution.phase(q).velocity(2:end,1)),PAR);
        dalpha = [dalpha;double(dalpha1)];
        dbeta = [dbeta;double(dbeta1)];
        taup = [taup;r.solution.phase(q).taup(2:end)];
		Fc = contact_forces(r.solution.phase(q),PAR,0);
		Fx3 = [Fx3;Fc(2:end,1)];
		Fy3 = [Fy3;Fc(2:end,2)];
        
    else
        theta = [theta;r.solution.phase(q).position(2:end,3)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end,3)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).position(2:end,1)];
        beta = [beta;r.solution.phase(q).position(2:end,2)];
        tau1 = [tau1;r.solution.phase(q).control(2:end,1)];
        tau2 = [tau2;r.solution.phase(q).control(2:end,2)];
        tau3 = [tau3;r.solution.phase(q).control(2:end,3)];
        tau4 = [tau4;r.solution.phase(q).control(2:end,4)];
        obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,3).^ind+r.solution.phase(q).control(2:end,4).^ind)];
        
        dalpha = [dalpha;r.solution.phase(q).velocity(2:end,1)];
        dbeta = [dbeta;r.solution.phase(q).velocity(2:end,2)];
        taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fx3 = [Fx3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fy3 = [Fy3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
        x = [x;(r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end)];
    end
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.tau11 = tau1;
out.tau21 = tau2;
out.tau12 = tau3;
out.tau22 = tau4;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.obj = obj;
out.tau_s = out.tau11-out.tau12;
out.tau_e = out.tau21-out.tau22;
out.x = x;

function out = open_MTG(r,PAR)

nsim = numel(r.solution.phase);
nsize = 0;
for q=1:nsim
    nsize = nsize + numel(r.solution.phase(q).position(:,1)) - 1;
end
nsize = nsize+1;
theta = r.solution.phase(1).position(1)+zeros(nsize,1);
dtheta = r.solution.phase(1).velocity(1)+zeros(nsize,1);
time = r.solution.phase(1).time(1)+zeros(nsize,1);
alpha = r.solution.phase(1).control(1,3)+zeros(nsize,1);
beta = r.solution.phase(1).control(1,4)+zeros(nsize,1);
act_se = r.solution.phase(1).control(1,1)+zeros(nsize,1);
act_ee = r.solution.phase(1).control(1,2)+zeros(nsize,1);
act_sf = r.solution.phase(1).control(1,5)+zeros(nsize,1);
act_ef = r.solution.phase(1).control(1,6)+zeros(nsize,1);
tau_se = r.solution.phase(1).control(1,8)+zeros(nsize,1);
tau_ee = r.solution.phase(1).control(1,9)+zeros(nsize,1);
tau_sf = r.solution.phase(1).control(1,10)+zeros(nsize,1);
tau_ef = r.solution.phase(1).control(1,11)+zeros(nsize,1);
tau_sp = zeros(nsize,1);
tau_ep = zeros(nsize,1);
ind = r.Options.Indice;
obj = r.solution.objective+zeros(nsim+1,1);
dalpha = 0+zeros(nsize,1);
dbeta = 0+zeros(nsize,1);
Fx3 = 0+zeros(nsize,1);
Fy3 = 0+zeros(nsize,1);
x = 0+zeros(nsize,1);

taup = r.solution.phase(1).taup(1)+zeros(nsize,1);
isize = 2;
for q=1:nsim
    zsize = isize+numel(r.solution.phase(q).position(2:end,1))-1;
    if rem(q,2)
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).control(2:end,3);
        beta(isize:zsize)   = r.solution.phase(q).control(2:end,4);
        act_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).control(2:end,5);
        act_ef(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,8);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,9);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,10);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,11);
        x(isize:zsize)      = (r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(end);
% 		if q == 1
			obj(q+1) = trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,8).^ind+r.solution.phase(q).control(:,9).^ind);
            
%         else
% 			obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,8).^ind+r.solution.phase(q).control(2:end,9).^ind)];
%         end
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end,1)),vect(r.solution.phase(q).velocity(2:end,1)),PAR);
        dalpha(isize:zsize) = double(dalpha1);
        dbeta(isize:zsize)  = double(dbeta1);
        taup(isize:zsize)   = r.solution.phase(q).taup(2:end);
		Fc = contact_forces(r.solution.phase(q),PAR,0);
		Fx3(isize:zsize) = Fc(2:end,1);
		Fy3(isize:zsize) = Fc(2:end,2);
        
    else
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,3);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,3);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        beta(isize:zsize)   = r.solution.phase(q).position(2:end,2);
        act_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).control(2:end,3);
        act_ef(isize:zsize) = r.solution.phase(q).control(2:end,4);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,5);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,7);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,8);
        obj(q+1) = trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,5).^ind+r.solution.phase(q).control(2:end,6).^ind);
        
        dalpha(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        dbeta(isize:zsize)  = r.solution.phase(q).velocity(2:end,2);
        taup(isize:zsize)   = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fx3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fy3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
        x(isize:zsize) = (r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end);
    end
    isize = zsize+1;
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.act_se = act_se;
out.act_ee = act_ee;
out.act_sf = act_sf;
out.act_ef = act_ef;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.obj = obj;
out.tau_se = tau_se;
out.tau_ee = tau_ee;
out.tau_sf = tau_sf;
out.tau_ef = tau_ef;
[ap1, bt1] = dof_conversion(vect(alpha),vect(beta),'Type', 'angle');
[dap1, dbt1] = dof_conversion(vect(dalpha),vect(dbeta),'Type', 'angvel');
[~,~,~,TP] = MTG_smooth(vect(act_sf),vect(act_se),vect(act_ef),vect(act_ee),ap1,bt1,dap1,dbt1);
out.tau_sp = -double(TP(1));
out.tau_ep = -double(TP(2));
out.tau_s = tau_se-tau_sf+2*out.tau_sp;
out.tau_e = tau_ee-tau_ef+2*out.tau_ep;
out.x = x;

function out = closed_MTG(r,PAR)

nsim = numel(r.solution.phase);
nsize = 0;
for q=1:nsim
    nsize = nsize + numel(r.solution.phase(q).position(:,1)) - 1;
end
nsize = nsize+1;
Mi = PAR.Mi;
Ci = PAR.Ci;
Kte = PAR.const;
Jtotal = PAR.Jmt;
Btotal = PAR.Bt;
Kmotor = PAR.rt*PAR.Kt;

theta = r.solution.phase(1).position(1)+zeros(nsize,1);
dtheta = r.solution.phase(1).velocity(1)+zeros(nsize,1);
time = r.solution.phase(1).time(1)+zeros(nsize,1);
alpha = r.solution.phase(1).control(1,3)+zeros(nsize,1);
beta = r.solution.phase(1).control(1,4)+zeros(nsize,1);
act_se = r.solution.phase(1).control(1,1)+zeros(nsize,1);
act_ee = r.solution.phase(1).control(1,2)+zeros(nsize,1);
act_sf = r.solution.phase(1).control(1,5)+zeros(nsize,1);
act_ef = r.solution.phase(1).control(1,6)+zeros(nsize,1);
tau_se = r.solution.phase(1).control(1,8)+zeros(nsize,1);
tau_ee = r.solution.phase(1).control(1,9)+zeros(nsize,1);
tau_sf = r.solution.phase(1).control(1,10)+zeros(nsize,1);
tau_ef = r.solution.phase(1).control(1,11)+zeros(nsize,1);
dveli = r.solution.phase(1).state(1,1)+zeros(nsize,1);
ierr = r.solution.phase(1).state(1,2)+zeros(nsize,1);
ind = r.Options.Indice;
obj = r.solution.objective;
objA = r.solution.objective+zeros(nsim+1,1);
objT = r.solution.objective+zeros(nsim+1,1);
objC = r.solution.objective+zeros(nsim+1,1);
dalpha = 0+zeros(nsize,1);
dbeta = 0+zeros(nsize,1);
Fx3 = 0+zeros(nsize,1);
Fy3 = 0+zeros(nsize,1);
x = 0+zeros(nsize,1);

taup = r.solution.phase(1).taup(1)+zeros(nsize,1);
isize = 2;
for q=1:nsim
    zsize = isize+numel(r.solution.phase(q).position(2:end,1))-1;
    if rem(q,2)
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).control(2:end,3);
        beta(isize:zsize)   = r.solution.phase(q).control(2:end,4);
        act_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).control(2:end,5);
        act_ef(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,8);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,9);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,10);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,11);
        dveli(isize:zsize)  = r.solution.phase(q).state(2:end,1);
        ierr(isize:zsize)   = r.solution.phase(q).state(2:end,2);
        x(isize:zsize)      = (r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(isize-1);
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end,1)),vect(r.solution.phase(q).velocity(2:end,1)),PAR);
        dalpha(isize:zsize) = double(dalpha1);
        dbeta(isize:zsize)  = double(dbeta1);
        taup(isize:zsize)   = r.solution.phase(q).taup(2:end);
        Fc = contact_forces_r(PAR,time(isize-1:zsize),[alpha(isize-1:zsize),beta(isize-1:zsize),theta(isize-1:zsize)], ...
            [dalpha(isize-1:zsize),dbeta(isize-1:zsize),dtheta(isize-1:zsize)], ...
            [act_ef(isize-1:zsize),act_ee(isize-1:zsize)],[act_sf(isize-1:zsize),act_se(isize-1:zsize)],[],0);
		Fx3(isize-1:zsize-1) = Fc(1:end-1,1);
		Fy3(isize-1:zsize-1) = Fc(1:end-1,2);
        
    else
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,3);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,3);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        beta(isize:zsize)   = r.solution.phase(q).position(2:end,2);
        act_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).control(2:end,3);
        act_ef(isize:zsize) = r.solution.phase(q).control(2:end,4);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,5);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,7);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,8);
        dveli(isize:zsize)  = r.solution.phase(q).state(2:end,1);
        ierr(isize:zsize)   = r.solution.phase(q).state(2:end,2);
        
        dalpha(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        dbeta(isize:zsize)  = r.solution.phase(q).velocity(2:end,2);
        taup(isize:zsize)   = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fx3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fy3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
        x(isize:zsize) = (r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(isize-1);
    end
    objA(q+1) = trapz(time(isize:zsize),act_se(isize:zsize).^ind+act_ee(isize:zsize).^ind+act_sf(isize:zsize).^ind+act_ef(isize:zsize).^ind);
    objT(q+1) = trapz(time(isize:zsize),tau_se(isize:zsize).^ind+tau_ee(isize:zsize).^ind+tau_sf(isize:zsize).^ind+tau_ef(isize:zsize).^ind);
        
    ctr_law(isize:zsize) = Kte*(Jtotal*(dveli(isize:zsize)/PAR.Rr-dtheta(isize:zsize))+Btotal*ierr(isize:zsize));
    objC(q+1) = trapz(time(isize:zsize),ctr_law(isize:zsize).^ind);
    isize = zsize+1;
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.act_se = act_se;
out.act_ee = act_ee;
out.act_sf = act_sf;
out.act_ef = act_ef;
out.dveli = dveli;
out.ierr = ierr;
u = Kte*(Jtotal*(dveli/PAR.Rr-dtheta)+Btotal*ierr);
objC(1) = trapz(time,u.^ind);
objA(1) = trapz(time,act_se.^ind+act_ee.^ind+act_sf.^ind+act_ef.^ind);
objT(1) = trapz(time,tau_se.^ind+tau_ee.^ind+tau_sf.^ind+tau_ef.^ind);
out.imotor = u/Kmotor;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.obj = obj;
out.objC = objC;
out.objA = objA;
out.objT = objT;
out.tau_se = tau_se;
out.tau_ee = tau_ee;
out.tau_sf = tau_sf;
out.tau_ef = tau_ef;
[ap1, bt1] = dof_conversion(vect(alpha),vect(beta),'Type', 'angle');
[dap1, dbt1] = dof_conversion(vect(dalpha),vect(dbeta),'Type', 'angvel');
[~,~,~,TP] = MTG_smooth(vect(act_sf),vect(act_se),vect(act_ef),vect(act_ee),ap1,bt1,dap1,dbt1);
out.tau_sp = -double(TP(1));
out.tau_ep = -double(TP(2));
out.tau_s = tau_se-tau_sf+2*out.tau_sp;
out.tau_e = tau_ee-tau_ef+2*out.tau_ep;
out.x = x;

function out = closed_MTG_dyn(r,PAR)

nsim = numel(r.solution.phase);
nsize = 0;
for q=1:nsim
    nsize = nsize + numel(r.solution.phase(q).position(:,1)) - 1;
end
nsize = nsize+1;
Mi = PAR.Mi;
Ci = PAR.Ci;
Kte = PAR.const;
Jtotal = PAR.Jf;
Btotal = PAR.Bf+0.01;
Kmotor = PAR.rt*PAR.Kt;

theta = r.solution.phase(1).position(1)+zeros(nsize,1);
dtheta = r.solution.phase(1).velocity(1)+zeros(nsize,1);
time = r.solution.phase(1).time(1)+zeros(nsize,1);
alpha = r.solution.phase(1).control(1,3)+zeros(nsize,1);
beta = r.solution.phase(1).control(1,4)+zeros(nsize,1);
dct_se = r.solution.phase(1).control(1,1)+zeros(nsize,1);
dct_ee = r.solution.phase(1).control(1,2)+zeros(nsize,1);
dct_sf = r.solution.phase(1).control(1,5)+zeros(nsize,1);
dct_ef = r.solution.phase(1).control(1,6)+zeros(nsize,1);
act_se = r.solution.phase(1).state(1,3)+zeros(nsize,1);
act_ee = r.solution.phase(1).state(1,4)+zeros(nsize,1);
act_sf = r.solution.phase(1).state(1,5)+zeros(nsize,1);
act_ef = r.solution.phase(1).state(1,6)+zeros(nsize,1);
tau_se = r.solution.phase(1).control(1,8)+zeros(nsize,1);
tau_ee = r.solution.phase(1).control(1,9)+zeros(nsize,1);
tau_sf = r.solution.phase(1).control(1,10)+zeros(nsize,1);
tau_ef = r.solution.phase(1).control(1,11)+zeros(nsize,1);
dveli = r.solution.phase(1).state(1,1)+zeros(nsize,1);
ierr = r.solution.phase(1).state(1,2)+zeros(nsize,1);
mass = zeros(nsize,1);
ctr_law = zeros(nsize,1);
ind = r.Options.Indice;
obj = r.solution.objective;
objA = r.solution.objective+zeros(nsim+1,1);
objT = r.solution.objective+zeros(nsim+1,1);
objD = r.solution.objective+zeros(nsim+1,1);
objC = r.solution.objective+zeros(nsim+1,1);
dalpha = 0+zeros(nsize,1);
dbeta = 0+zeros(nsize,1);
Fx3 = 0+zeros(nsize,1);
Fy3 = 0+zeros(nsize,1);
x = 0+zeros(nsize,1);

taup = r.solution.phase(1).taup(1)+zeros(nsize,1);
isize = 2;
for q=1:nsim
    zsize = isize+numel(r.solution.phase(q).position(2:end,1))-1;
    if rem(q,2)
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).control(2:end,3);
        beta(isize:zsize)   = r.solution.phase(q).control(2:end,4);
        dct_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        dct_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        dct_sf(isize:zsize) = r.solution.phase(q).control(2:end,5);
        dct_ef(isize:zsize) = r.solution.phase(q).control(2:end,6);
        act_se(isize:zsize) = r.solution.phase(q).state(2:end,3);
        act_ee(isize:zsize) = r.solution.phase(q).state(2:end,4);
        act_sf(isize:zsize) = r.solution.phase(q).state(2:end,5);
        act_ef(isize:zsize) = r.solution.phase(q).state(2:end,6);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,8);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,9);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,10);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,11);
        dveli(isize:zsize)  = r.solution.phase(q).state(2:end,1);
        ierr(isize:zsize)   = r.solution.phase(q).state(2:end,2);
        x(isize:zsize)      = (r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(isize-1);
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end,1)),vect(r.solution.phase(q).velocity(2:end,1)),PAR);
        dalpha(isize:zsize) = double(dalpha1);
        dbeta(isize:zsize)  = double(dbeta1);
        taup(isize:zsize)   = r.solution.phase(q).taup(2:end);
        Fc = contact_forces_r(PAR,time(isize-1:zsize),[alpha(isize-1:zsize),beta(isize-1:zsize),theta(isize-1:zsize)], ...
            [dalpha(isize-1:zsize),dbeta(isize-1:zsize),dtheta(isize-1:zsize)], ...
            [act_ef(isize-1:zsize),act_ee(isize-1:zsize)],[act_sf(isize-1:zsize),act_se(isize-1:zsize)],[],0);
		Fx3(isize-1:zsize-1) = Fc(1:end-1,1);
		Fy3(isize-1:zsize-1) = Fc(1:end-1,2);
        mass(isize-1:zsize) = Fc(1:end,4);
        
    else
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,3);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,3);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        beta(isize:zsize)   = r.solution.phase(q).position(2:end,2);
        dct_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        dct_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        dct_sf(isize:zsize) = r.solution.phase(q).control(2:end,3);
        dct_ef(isize:zsize) = r.solution.phase(q).control(2:end,4);
        act_se(isize:zsize) = r.solution.phase(q).state(2:end,3);
        act_ee(isize:zsize) = r.solution.phase(q).state(2:end,4);
        act_sf(isize:zsize) = r.solution.phase(q).state(2:end,5);
        act_ef(isize:zsize) = r.solution.phase(q).state(2:end,6);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,5);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,7);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,8);
        dveli(isize:zsize)  = r.solution.phase(q).state(2:end,1);
        ierr(isize:zsize)   = r.solution.phase(q).state(2:end,2);
        x(isize:zsize)      = (r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(isize-1);
        
        dalpha(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        dbeta(isize:zsize)  = r.solution.phase(q).velocity(2:end,2);
        taup(isize:zsize)   = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fx3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fy3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
        
        mass(isize:zsize) = equivalent_mass(alpha(isize:zsize),beta(isize:zsize),theta(isize:zsize), ...
            dalpha(isize:zsize),dbeta(isize:zsize),dtheta(isize:zsize),PAR);
    end
    objA(q+1) = trapz(time(isize:zsize),act_se(isize:zsize).^ind+act_ee(isize:zsize).^ind+act_sf(isize:zsize).^ind+act_ef(isize:zsize).^ind);
    objD(q+1) = trapz(time(isize:zsize),dct_se(isize:zsize).^ind+dct_ee(isize:zsize).^ind+dct_sf(isize:zsize).^ind+dct_ef(isize:zsize).^ind);
    objT(q+1) = trapz(time(isize:zsize),tau_se(isize:zsize).^ind+tau_ee(isize:zsize).^ind+tau_sf(isize:zsize).^ind+tau_ef(isize:zsize).^ind);
        
    ctr_law(isize:zsize) = Kte*(Jtotal*(dveli(isize:zsize)/PAR.Rr-dtheta(isize:zsize))+Btotal*ierr(isize:zsize));
    objC(q+1) = trapz(time(isize:zsize),ctr_law(isize:zsize).^ind);
    isize = zsize+1;
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.dct_se = dct_se;
out.dct_ee = dct_ee;
out.dct_sf = dct_sf;
out.dct_ef = dct_ef;
out.act_se = act_se;
out.act_ee = act_ee;
out.act_sf = act_sf;
out.act_ef = act_ef;
out.mass = mass;
out.dveli = dveli;
out.ierr = ierr;
u = Kte*(Jtotal*(dveli/PAR.Rr-dtheta)+Btotal*ierr);
[ap1, bt1] = dof_conversion(vect(alpha),vect(beta),'Type', 'angle');
[dap1, dbt1] = dof_conversion(vect(dalpha),vect(dbeta),'Type', 'angvel');
[TT,~,~,TP] = MTG_smooth(vect(act_sf),vect(act_se),vect(act_ef),vect(act_ee),ap1,bt1,dap1,dbt1);
out.tau_sp = -double(TP(1));
out.tau_ep = -double(TP(2));
objC(1) = trapz(time,u.^ind);
objA(1) = trapz(time,act_se.^ind+act_ee.^ind+act_sf.^ind+act_ef.^ind);
objD(1) = trapz(time,dct_se.^ind+dct_ee.^ind+dct_sf.^ind+dct_ef.^ind);
objT(1) = trapz(time,double(TT(1)).^ind+double(TT(2)).^ind+double(TT(3)).^ind+double(TT(4)).^ind);
out.imotor = u/Kmotor;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.obj = obj;
out.objA = objA;
out.objD = objD;
out.objT = objT;
out.objC = objC;
out.ctr_law = ctr_law;
out.tau_se = tau_se;
out.tau_ee = tau_ee;
out.tau_sf = tau_sf;
out.tau_ef = tau_ef;
out.tau_s = tau_se-tau_sf+2*out.tau_sp;
out.tau_e = tau_ee-tau_ef+2*out.tau_ep;
out.x = x;

function out = open_MTG_dyn(r,PAR)

nsim = numel(r.solution.phase);
nsize = 0;
for q=1:nsim
    nsize = nsize + numel(r.solution.phase(q).position(:,1)) - 1;
end
nsize = nsize+1;
theta = r.solution.phase(1).position(1)+zeros(nsize,1);
dtheta = r.solution.phase(1).velocity(1)+zeros(nsize,1);
time = r.solution.phase(1).time(1)+zeros(nsize,1);
alpha = r.solution.phase(1).control(1,3)+zeros(nsize,1);
beta = r.solution.phase(1).control(1,4)+zeros(nsize,1);
act_se = r.solution.phase(1).state(1,1)+zeros(nsize,1);
act_ee = r.solution.phase(1).state(1,2)+zeros(nsize,1);
act_sf = r.solution.phase(1).state(1,3)+zeros(nsize,1);
act_ef = r.solution.phase(1).state(1,4)+zeros(nsize,1);
dct_se = r.solution.phase(1).control(1,1)+zeros(nsize,1);
dct_ee = r.solution.phase(1).control(1,2)+zeros(nsize,1);
dct_sf = r.solution.phase(1).control(1,5)+zeros(nsize,1);
dct_ef = r.solution.phase(1).control(1,6)+zeros(nsize,1);
tau_se = r.solution.phase(1).control(1,8)+zeros(nsize,1);
tau_ee = r.solution.phase(1).control(1,9)+zeros(nsize,1);
tau_sf = r.solution.phase(1).control(1,10)+zeros(nsize,1);
tau_ef = r.solution.phase(1).control(1,11)+zeros(nsize,1);
ind = r.Options.Indice;
obj = r.solution.objective;
objA = r.solution.objective+zeros(nsim+1,1);
objD = r.solution.objective+zeros(nsim+1,1);
objT = r.solution.objective+zeros(nsim+1,1);
dalpha = 0+zeros(nsize,1);
dbeta = 0+zeros(nsize,1);
Fx3 = 0+zeros(nsize,1);
Fy3 = 0+zeros(nsize,1);
mass = zeros(nsize,1);
x = 0+zeros(nsize,1);

taup = r.solution.phase(1).taup(1)+zeros(nsize,1);
isize = 2;
for q=1:nsim
    zsize = isize+numel(r.solution.phase(q).position(2:end,1))-1;
    if rem(q,2)
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).control(2:end,3);
        beta(isize:zsize)   = r.solution.phase(q).control(2:end,4);
        act_se(isize:zsize) = r.solution.phase(q).state(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).state(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).state(2:end,3);
        act_ef(isize:zsize) = r.solution.phase(q).state(2:end,4);
        dct_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        dct_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        dct_sf(isize:zsize) = r.solution.phase(q).control(2:end,5);
        dct_ef(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,8);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,9);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,10);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,11);
        x(isize:zsize)      = (r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(isize-1);
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end,1)),vect(r.solution.phase(q).velocity(2:end,1)),PAR);
        dalpha(isize:zsize) = double(dalpha1);
        dbeta(isize:zsize)  = double(dbeta1);
        taup(isize:zsize)   = r.solution.phase(q).taup(2:end);
        Fc = contact_forces_r(PAR,time(isize-1:zsize),[alpha(isize-1:zsize),beta(isize-1:zsize),theta(isize-1:zsize)], ...
            [dalpha(isize-1:zsize),dbeta(isize-1:zsize),dtheta(isize-1:zsize)], ...
            [act_ef(isize-1:zsize),act_ee(isize-1:zsize)],[act_sf(isize-1:zsize),act_se(isize-1:zsize)],[],0);
% 		Fc = contact_forces(r.solution.phase(q),PAR,0);
		Fx3(isize-1:zsize-1) = Fc(1:end-1,1);
		Fy3(isize-1:zsize-1) = Fc(1:end-1,2);
        mass(isize-1:zsize) = Fc(:,4);
    else
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,3);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,3);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        beta(isize:zsize)   = r.solution.phase(q).position(2:end,2);
        act_se(isize:zsize) = r.solution.phase(q).state(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).state(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).state(2:end,3);
        act_ef(isize:zsize) = r.solution.phase(q).state(2:end,4);
        dct_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        dct_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        dct_sf(isize:zsize) = r.solution.phase(q).control(2:end,3);
        dct_ef(isize:zsize) = r.solution.phase(q).control(2:end,4);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,5);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,7);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,8);
        
        dalpha(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        dbeta(isize:zsize)  = r.solution.phase(q).velocity(2:end,2);
        taup(isize:zsize)   = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fx3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fy3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
        mass(isize:zsize) = equivalent_mass(alpha(isize:zsize),beta(isize:zsize),theta(isize:zsize), ...
            dalpha(isize:zsize),dbeta(isize:zsize),dtheta(isize:zsize),PAR);
        x(isize:zsize) = (r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(isize-1);
    end
    objA(q+1) = trapz(time(isize:zsize),act_se(isize:zsize).^ind+act_ee(isize:zsize).^ind+act_sf(isize:zsize).^ind+act_ef(isize:zsize).^ind);
    objD(q+1) = trapz(time(isize:zsize),dct_se(isize:zsize).^ind+dct_ee(isize:zsize).^ind+dct_sf(isize:zsize).^ind+dct_ef(isize:zsize).^ind);
    objT(q+1) = trapz(time(isize:zsize),tau_se(isize:zsize).^ind+tau_ee(isize:zsize).^ind+tau_sf(isize:zsize).^ind+tau_ef(isize:zsize).^ind);
    isize = zsize+1;
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.act_se = act_se;
out.act_ee = act_ee;
out.act_sf = act_sf;
out.act_ef = act_ef;
out.dct_se = dct_se;
out.dct_ee = dct_ee;
out.dct_sf = dct_sf;
out.dct_ef = dct_ef;
out.mass = mass;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.obj = obj;
[ap1, bt1] = dof_conversion(vect(alpha),vect(beta),'Type', 'angle');
[dap1, dbt1] = dof_conversion(vect(dalpha),vect(dbeta),'Type', 'angvel');
[TT,~,~,TP] = MTG_smooth(vect(act_sf),vect(act_se),vect(act_ef),vect(act_ee),ap1,bt1,dap1,dbt1);
out.tau_sp = -double(TP(1));
out.tau_ep = -double(TP(2));
objA(1) = trapz(time,act_se.^ind+act_ee.^ind+act_sf.^ind+act_ef.^ind);
objD(1) = trapz(time,dct_se.^ind+dct_ee.^ind+dct_sf.^ind+dct_ef.^ind);
objT(1) = trapz(time,double(TT(1)).^ind+double(TT(2)).^ind+double(TT(3)).^ind+double(TT(4)).^ind);
out.objA = objA;
out.objD = objD;
out.objT = objT;
out.tau_se = tau_se;
out.tau_ee = tau_ee;
out.tau_sf = tau_sf;
out.tau_ef = tau_ef;
out.tau_s = tau_se-tau_sf+2*out.tau_sp;
out.tau_e = tau_ee-tau_ef+2*out.tau_ep;
out.x = x;

function out = open_MTG_dyn_test(r,PAR)

nsim = numel(r.solution.phase);
nsize = 0;
for q=1:nsim
    nsize = nsize + numel(r.solution.phase(q).position(:,1)) - 1;
end
nsize = nsize+1;
alpha = r.solution.phase(1).position(1,1)+zeros(nsize,1);
beta  = r.solution.phase(1).position(1,2)+zeros(nsize,1);
theta = r.solution.phase(1).position(1,3)+zeros(nsize,1);
dalpha = r.solution.phase(1).velocity(1,1)+zeros(nsize,1);
dbeta  = r.solution.phase(1).velocity(1,2)+zeros(nsize,1);
dtheta = r.solution.phase(1).velocity(1,3)+zeros(nsize,1);
time = r.solution.phase(1).time(1)+zeros(nsize,1);
act_se = r.solution.phase(1).state(1,1)+zeros(nsize,1);
act_ee = r.solution.phase(1).state(1,2)+zeros(nsize,1);
act_sf = r.solution.phase(1).state(1,3)+zeros(nsize,1);
act_ef = r.solution.phase(1).state(1,4)+zeros(nsize,1);
dct_se = r.solution.phase(1).control(1,1)+zeros(nsize,1);
dct_ee = r.solution.phase(1).control(1,2)+zeros(nsize,1);
dct_sf = r.solution.phase(1).control(1,5)+zeros(nsize,1);
dct_ef = r.solution.phase(1).control(1,6)+zeros(nsize,1);
tau_se = r.solution.phase(1).control(1,7)+zeros(nsize,1);
tau_ee = r.solution.phase(1).control(1,8)+zeros(nsize,1);
tau_sf = r.solution.phase(1).control(1,9)+zeros(nsize,1);
tau_ef = r.solution.phase(1).control(1,10)+zeros(nsize,1);
Fx3 = r.solution.phase(1).control(1,3)+zeros(nsize,1);
Fy3 = r.solution.phase(1).control(1,4)+zeros(nsize,1);
ind = r.Options.Indice;
obj = r.solution.objective+zeros(nsim+1,1);
x = 0+zeros(nsize,1);

taup = r.solution.phase(1).taup(1)+zeros(nsize,1);
isize = 2;
for q=1:nsim
    zsize = isize+numel(r.solution.phase(q).position(2:end,1))-1;
    if rem(q,2)
        alpha(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        beta(isize:zsize)   = r.solution.phase(q).position(2:end,2);
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,3);
        dalpha(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        dbeta(isize:zsize)  = r.solution.phase(q).velocity(2:end,2);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,3);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        Fx3(isize:zsize)    = r.solution.phase(q).control(2:end,3);
        Fy3(isize:zsize)    = r.solution.phase(q).control(2:end,4);
        act_se(isize:zsize) = r.solution.phase(q).state(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).state(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).state(2:end,3);
        act_ef(isize:zsize) = r.solution.phase(q).state(2:end,4);
        dct_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        dct_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        dct_sf(isize:zsize) = r.solution.phase(q).control(2:end,5);
        dct_ef(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,7);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,8);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,9);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,10);
        x(isize:zsize)      = (r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end);
% 		if q == 1
			obj(q+1) = trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,8).^ind+r.solution.phase(q).control(:,9).^ind);
            
%         else
% 			obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,8).^ind+r.solution.phase(q).control(2:end,9).^ind)];
%         end
		
%         [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end,1)),vect(r.solution.phase(q).velocity(2:end,1)),PAR);
%         dalpha(isize:zsize) = double(dalpha1);
%         dbeta(isize:zsize)  = double(dbeta1);
        taup(isize:zsize)   = r.solution.phase(q).taup(2:end);
% 		Fc = contact_forces(r.solution.phase(q),PAR,0);
% 		Fx3(isize:zsize) = Fc(2:end,1);
% 		Fy3(isize:zsize) = Fc(2:end,2);
        
    else
        theta(isize:zsize)  = r.solution.phase(q).position(2:end,3);
        dtheta(isize:zsize) = r.solution.phase(q).velocity(2:end,3);
        time(isize:zsize)   = r.solution.phase(q).time(2:end);
        alpha(isize:zsize)  = r.solution.phase(q).position(2:end,1);
        beta(isize:zsize)   = r.solution.phase(q).position(2:end,2);
        act_se(isize:zsize) = r.solution.phase(q).state(2:end,1);
        act_ee(isize:zsize) = r.solution.phase(q).state(2:end,2);
        act_sf(isize:zsize) = r.solution.phase(q).state(2:end,3);
        act_ef(isize:zsize) = r.solution.phase(q).state(2:end,4);
        dct_se(isize:zsize) = r.solution.phase(q).control(2:end,1);
        dct_ee(isize:zsize) = r.solution.phase(q).control(2:end,2);
        dct_sf(isize:zsize) = r.solution.phase(q).control(2:end,3);
        dct_ef(isize:zsize) = r.solution.phase(q).control(2:end,4);
        tau_se(isize:zsize) = r.solution.phase(q).control(2:end,5);
        tau_ee(isize:zsize) = r.solution.phase(q).control(2:end,6);
        tau_sf(isize:zsize) = r.solution.phase(q).control(2:end,7);
        tau_ef(isize:zsize) = r.solution.phase(q).control(2:end,8);
        obj(q+1) = trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,5).^ind+r.solution.phase(q).control(2:end,6).^ind);
        
        dalpha(isize:zsize) = r.solution.phase(q).velocity(2:end,1);
        dbeta(isize:zsize)  = r.solution.phase(q).velocity(2:end,2);
        taup(isize:zsize)   = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fx3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
		Fy3(isize:zsize)    = zeros(numel(r.solution.phase(q).time(2:end)),1);
        x(isize:zsize) = (r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end);
    end
    isize = zsize+1;
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.act_se = act_se;
out.act_ee = act_ee;
out.act_sf = act_sf;
out.act_ef = act_ef;
out.dct_se = dct_se;
out.dct_ee = dct_ee;
out.dct_sf = dct_sf;
out.dct_ef = dct_ef;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.obj = obj;
out.tau_se = tau_se;
out.tau_ee = tau_ee;
out.tau_sf = tau_sf;
out.tau_ef = tau_ef;
[ap1, bt1] = dof_conversion(vect(alpha),vect(beta),'Type', 'angle');
[dap1, dbt1] = dof_conversion(vect(dalpha),vect(dbeta),'Type', 'angvel');
[~,~,~,TP] = MTG_smooth(vect(act_sf),vect(act_se),vect(act_ef),vect(act_ee),ap1,bt1,dap1,dbt1);
out.tau_sp = -double(TP(1));
out.tau_ep = -double(TP(2));
out.tau_s = tau_se-tau_sf+2*out.tau_sp;
out.tau_e = tau_ee-tau_ef+2*out.tau_ep;
out.x = x;


function out = closed_PID_FirstOrder(r,PAR)

nsim = numel(r.solution.phase);
theta = r.solution.phase(1).position(1);
dtheta = r.solution.phase(1).velocity(1);
time = r.solution.phase(1).time(1);
alpha = r.solution.phase(1).control(1,3);
beta = r.solution.phase(1).control(1,4);
tau1 = r.solution.phase(1).control(1,1);
tau2 = r.solution.phase(1).control(1,2);
dalpha = 0;
dbeta = 0;
dveli = 0;
ierr = 0;
imotor = 0;
dveli = r.solution.phase(1).state(1,1);
ierr = r.solution.phase(1).state(1,2);
imotor = r.solution.phase(1).state(1,3);
derr = r.solution.phase(1).state(1,4);
taup = r.solution.phase(1).control(1,5);
Fx3 = 0;
Fy3 = 0;
x = 0;

for q=1:nsim
    
    if rem(q,2)
        theta = [theta;r.solution.phase(q).position(2:end)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).control(2:end,3)];
        beta = [beta;r.solution.phase(q).control(2:end,4)];
        tau1 = [tau1;r.solution.phase(q).control(2:end,1)];
        tau2 = [tau2;r.solution.phase(q).control(2:end,2)];
        x = [x;(r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(end)];
        
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end)),vect(r.solution.phase(q).velocity(2:end)),PAR);
        dalpha = [dalpha;double(dalpha1)];
        dbeta = [dbeta;double(dbeta1)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
        imotor = [imotor;r.solution.phase(q).state(2:end,3)];
        derr = [derr;r.solution.phase(q).state(2:end,4)];
        taup = [taup;r.solution.phase(q).control(2:end,5)];
		Fc = contact_forces(r.solution.phase(q),PAR,1);
		Fx3 = [Fx3;Fc(2:end,1)];
		Fy3 = [Fy3;Fc(2:end,2)];
        
    else
        theta = [theta;r.solution.phase(q).position(2:end,3)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end,3)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).position(2:end,1)];
        beta = [beta;r.solution.phase(q).position(2:end,2)];
        tau1 = [tau1;r.solution.phase(q).control(2:end,1)];
        tau2 = [tau2;r.solution.phase(q).control(2:end,2)];
        x = [x;(r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end)];
        
        dalpha = [dalpha;r.solution.phase(q).velocity(2:end,1)];
        dbeta = [dbeta;r.solution.phase(q).velocity(2:end,2)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
        imotor = [imotor;r.solution.phase(q).state(2:end,3)];
        derr = [derr;r.solution.phase(q).state(2:end,4)];
        taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fx3 = [Fx3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fy3 = [Fy3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
    end
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.tau1 = tau1;
out.tau2 = tau2;
out.dveli = dveli;
out.ierr = ierr;
out.imotor = imotor;
out.derr = derr;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.x = x;

function out = closed_PID_Observer(r,PAR)

nsim = numel(r.solution.phase);
theta = r.solution.phase(1).position(1);
dtheta = r.solution.phase(1).velocity(1);
time = r.solution.phase(1).time(1);
alpha = r.solution.phase(1).control(1,3);
beta = r.solution.phase(1).control(1,4);
tau11 = r.solution.phase(1).control(1,1);
tau12 = r.solution.phase(1).control(1,5);
tau21 = r.solution.phase(1).control(1,2);
tau22 = r.solution.phase(1).control(1,6);
ind = r.Options.Indice;
obj = r.solution.objective;
objC = 0;
dalpha = 0;
dbeta = 0;
dveli = 0;
ierr = 0;
imotor = 0;
% ddtheta = 0;
dveli = r.solution.phase(1).state(1,1);
ierr = r.solution.phase(1).state(1,2);
imotor = r.solution.phase(1).state(1,3);
derr = r.solution.phase(1).state(1,4);
taup = r.solution.phase(1).control(1,7);
Fx3 = 0;
Fy3 = 0;
rderr = 0;
x = 0;
% CtrLaw = 0;

Mi = PAR.Mi;
Ci = PAR.Ci;
Kte = PAR.const;
Jtotal = PAR.Jmt;
Btotal = PAR.Bt;
Kmotor = PAR.rt*PAR.Kt;
Ke = PAR.rt*PAR.Ke;
Ra = PAR.Ra;
La = PAR.La;
Td = PAR.Td;
Pr = PAR.R;
Rr = PAR.Rr;

for q=1:nsim
    
    if rem(q,2)
        theta = [theta;r.solution.phase(q).position(2:end)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).control(2:end,3)];
        beta = [beta;r.solution.phase(q).control(2:end,4)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,5)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,6)];
		if q == 1
			obj = [obj;trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,1).^ind+r.solution.phase(q).control(:,2).^ind+r.solution.phase(q).control(:,5).^ind+r.solution.phase(q).control(:,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time,Kmotor*r.solution.phase(q).state(:,3).^ind)];
%             objC = [objC;trapz(r.solution.phase(q).time,Kmotor*r.solution.phase(q).state(:,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
		else
			obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,5).^ind+r.solution.phase(q).control(2:end,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)];
%             objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        end
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end)),vect(r.solution.phase(q).velocity(2:end)),PAR);
        dalpha = [dalpha;double(dalpha1)];
        dbeta = [dbeta;double(dbeta1)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
        imotor = [imotor;r.solution.phase(q).state(2:end,3)];
        derr = [derr;r.solution.phase(q).state(2:end,4)];
        taup = [taup;r.solution.phase(q).control(2:end,7)];
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fc = contact_forces(r.solution.phase(q),PAR,1);
		Fx3 = [Fx3;Fc(2:end,1)];
		Fy3 = [Fy3;Fc(2:end,2)];
        x = [x;(r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(end)];
        
        ddtheta = Fc(2:end,3);
        ddveli = (1/Mi)*(r.solution.phase(q).control(2:end,7)/Pr-Ci*r.solution.phase(q).state(2:end,1));
        rderr = [rderr;ddveli/Rr-ddtheta];
        
    else
        theta = [theta;r.solution.phase(q).position(2:end,3)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end,3)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).position(2:end,1)];
        beta = [beta;r.solution.phase(q).position(2:end,2)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,3)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,4)];
        obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,3).^ind+r.solution.phase(q).control(2:end,4).^ind)];
        objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)];
%         objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        
        dalpha = [dalpha;r.solution.phase(q).velocity(2:end,1)];
        dbeta = [dbeta;r.solution.phase(q).velocity(2:end,2)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
        imotor = [imotor;r.solution.phase(q).state(2:end,3)];
        derr = [derr;r.solution.phase(q).state(2:end,4)];
		if size(r.solution.phase(q).control,2)>4
			taup = [taup;r.solution.phase(q).control(2:end,5)];
		else
			taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		end
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fx3 = [Fx3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fy3 = [Fy3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
        x = [x;(r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end)];
        
        ddtheta = acceleration(r.solution.phase(q),PAR,1);
        ddveli = (1/Mi)*(-Ci*r.solution.phase(q).state(2:end,1));
        rderr = [rderr;ddveli/Rr-ddtheta(2:end)];
    end
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.tau11 = tau11;
out.tau21 = tau21;
out.tau12 = tau12;
out.tau22 = tau22;
out.obj = obj;
objC(1) = trapz(out.time,Kmotor*imotor.^ind);
% objC(1) = sum(objC(2:end));
out.objC = objC;
out.dveli = dveli;
out.ierr = ierr;
out.imotor = imotor;
out.derr = derr;
out.rderr = rderr;
out.err = dveli/Rr-dtheta;
% out.ddtheta = ddtheta;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.tau_s = out.tau11-out.tau12;
out.tau_e = out.tau21-out.tau22;
out.ctrlaw = Kte*((Jtotal*Ra+La*Btotal)*(dveli/Rr-dtheta)+(Btotal*Ra+Ke)*ierr+La*Jtotal*(1/PAR.Td)*((dveli/Rr-dtheta)-derr))/Kmotor;
out.x = x;

function out = closed_PIDSAT_Observer(r,PAR)

nsim = numel(r.solution.phase);
theta = r.solution.phase(1).position(1);
dtheta = r.solution.phase(1).velocity(1);
time = r.solution.phase(1).time(1);
alpha = r.solution.phase(1).control(1,3);
beta = r.solution.phase(1).control(1,4);
tau11 = r.solution.phase(1).control(1,1);
tau12 = r.solution.phase(1).control(1,5);
tau21 = r.solution.phase(1).control(1,2);
tau22 = r.solution.phase(1).control(1,6);
u = r.solution.phase(1).control(1,8);
vp = r.solution.phase(1).control(1,9);
vn = r.solution.phase(1).control(1,10);
ind = r.Options.Indice;
obj = r.solution.objective;
objC = 0;
dalpha = 0;
dbeta = 0;
dveli = 0;
ierr = 0;
imotor = 0;
% ddtheta = 0;
dveli = r.solution.phase(1).state(1,1);
ierr = r.solution.phase(1).state(1,2);
imotor = r.solution.phase(1).state(1,3);
derr = r.solution.phase(1).state(1,4);
taup = r.solution.phase(1).control(1,7);
Fx3 = 0;
Fy3 = 0;
rderr = 0;
x = 0;
% CtrLaw = 0;

Mi = PAR.Mi;
Ci = PAR.Ci;
Kte = PAR.const;
Jtotal = PAR.Jmt;
Btotal = PAR.Bt;
Kmotor = PAR.rt*PAR.Kt;
Ke = PAR.rt*PAR.Ke;
Ra = PAR.Ra;
La = PAR.La;
Td = PAR.Td;
Pr = PAR.R;
Rr = PAR.Rr;

for q=1:nsim
    
    if rem(q,2)
        theta = [theta;r.solution.phase(q).position(2:end)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).control(2:end,3)];
        beta = [beta;r.solution.phase(q).control(2:end,4)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,5)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,6)];
        u = [u;r.solution.phase(q).control(2:end,8)];
        vp = [vp;r.solution.phase(q).control(2:end,9)];
        vn = [vn;r.solution.phase(q).control(2:end,10)];
		if q == 1
			obj = [obj;trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,1).^ind+r.solution.phase(q).control(:,2).^ind+r.solution.phase(q).control(:,5).^ind+r.solution.phase(q).control(:,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time,Kmotor*r.solution.phase(q).state(:,3).^ind)];
%             objC = [objC;trapz(r.solution.phase(q).time,Kmotor*r.solution.phase(q).state(:,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
		else
			obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,5).^ind+r.solution.phase(q).control(2:end,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)];
%             objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        end
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end)),vect(r.solution.phase(q).velocity(2:end)),PAR);
        dalpha = [dalpha;double(dalpha1)];
        dbeta = [dbeta;double(dbeta1)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
        imotor = [imotor;r.solution.phase(q).state(2:end,3)];
        derr = [derr;r.solution.phase(q).state(2:end,4)];
        taup = [taup;r.solution.phase(q).control(2:end,7)];
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fc = contact_forces(r.solution.phase(q),PAR,1);
		Fx3 = [Fx3;Fc(2:end,1)];
		Fy3 = [Fy3;Fc(2:end,2)];
        x = [x;(r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(end)];
        
        ddtheta = Fc(2:end,3);
        ddveli = (1/Mi)*(r.solution.phase(q).control(2:end,7)/Pr-Ci*r.solution.phase(q).state(2:end,1));
        rderr = [rderr;ddveli/Rr-ddtheta];
        
    else
        theta = [theta;r.solution.phase(q).position(2:end,3)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end,3)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).position(2:end,1)];
        beta = [beta;r.solution.phase(q).position(2:end,2)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,3)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,4)];
        u = [u;r.solution.phase(q).control(2:end,5)];
        vp = [vp;r.solution.phase(q).control(2:end,6)];
        vn = [vn;r.solution.phase(q).control(2:end,7)];
        obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,3).^ind+r.solution.phase(q).control(2:end,4).^ind)];
        objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)];
%         objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        
        dalpha = [dalpha;r.solution.phase(q).velocity(2:end,1)];
        dbeta = [dbeta;r.solution.phase(q).velocity(2:end,2)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
        imotor = [imotor;r.solution.phase(q).state(2:end,3)];
        derr = [derr;r.solution.phase(q).state(2:end,4)];
		if size(r.solution.phase(q).control,2)>4
			taup = [taup;r.solution.phase(q).control(2:end,5)];
		else
			taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		end
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fx3 = [Fx3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fy3 = [Fy3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
        x = [x;(r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end)];
        
        ddtheta = acceleration(r.solution.phase(q),PAR,1);
        ddveli = (1/Mi)*(-Ci*r.solution.phase(q).state(2:end,1));
        rderr = [rderr;ddveli/Rr-ddtheta(2:end)];
    end
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.tau11 = tau11;
out.tau21 = tau21;
out.tau12 = tau12;
out.tau22 = tau22;
out.obj = obj;
objC(1) = trapz(out.time,Kmotor*imotor.^ind);
% objC(1) = sum(objC(2:end));
out.objC = objC;
out.dveli = dveli;
out.ierr = ierr;
out.imotor = imotor;
out.derr = derr;
out.rderr = rderr;
out.err = dveli/Rr-dtheta;
% out.ddtheta = ddtheta;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.tau_s = out.tau11-out.tau12;
out.tau_e = out.tau21-out.tau22;
out.ctrlaw = Kte*((Jtotal*Ra+La*Btotal)*(dveli/Rr-dtheta)+(Btotal*Ra+Ke)*ierr+La*Jtotal*(1/PAR.Td)*((dveli/Rr-dtheta)-derr))/Kmotor;
out.ctrlawU = u;
out.ctrlawV = vp-vn;
out.x = x;

function out = closed_PI_Observer(r,PAR)

nsim = numel(r.solution.phase);
theta = r.solution.phase(1).position(1);
dtheta = r.solution.phase(1).velocity(1);
time = r.solution.phase(1).time(1);
alpha = r.solution.phase(1).control(1,3);
beta = r.solution.phase(1).control(1,4);
tau11 = r.solution.phase(1).control(1,1);
tau12 = r.solution.phase(1).control(1,5);
tau21 = r.solution.phase(1).control(1,2);
tau22 = r.solution.phase(1).control(1,6);
u = r.solution.phase(1).control(1,8);
vp = r.solution.phase(1).control(1,9);
vn = r.solution.phase(1).control(1,10);
ind = r.Options.Indice;
obj = r.solution.objective;
objC = 0;
dalpha = 0;
dbeta = 0;
dveli = 0;
ierr = 0;
imotor = 0;
ddth = 0;
dveli = r.solution.phase(1).state(1,1);
ierr = r.solution.phase(1).state(1,2);
% imotor = r.solution.phase(1).state(1,3);
% derr = r.solution.phase(1).state(1,4);
taup = r.solution.phase(1).control(1,7);
Fx3 = 0;
Fy3 = 0;
rderr = 0;
x = 0;
% CtrLaw = 0;

Mi = PAR.Mi;
Ci = PAR.Ci;
Kte = PAR.const;
Jtotal = PAR.Jmf;
Btotal = PAR.Bf+0.01;
Kmotor = PAR.rt*PAR.Kt;
Ke = PAR.rt*PAR.Ke;
Ra = PAR.Ra;
La = PAR.La;
Td = PAR.Td;
Pr = PAR.R;
Rr = PAR.Rr;

for q=1:nsim
    
    if rem(q,2)
        theta = [theta;r.solution.phase(q).position(2:end)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).control(2:end,3)];
        beta = [beta;r.solution.phase(q).control(2:end,4)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,5)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,6)];
        u = [u;r.solution.phase(q).control(2:end,8)];
        vp = [vp;r.solution.phase(q).control(2:end,9)];
        vn = [vn;r.solution.phase(q).control(2:end,10)];
		if q == 1
			obj = [obj;trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,1).^ind+r.solution.phase(q).control(:,2).^ind+r.solution.phase(q).control(:,5).^ind+r.solution.phase(q).control(:,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,8).^ind)];
%             objC = [objC;trapz(r.solution.phase(q).time,Kmotor*r.solution.phase(q).state(:,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
		else
			obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,5).^ind+r.solution.phase(q).control(2:end,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,8).^ind)];
%             objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        end
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end)),vect(r.solution.phase(q).velocity(2:end)),PAR);
        dalpha = [dalpha;double(dalpha1)];
        dbeta = [dbeta;double(dbeta1)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
%         imotor = [imotor;r.solution.phase(q).state(2:end,3)];
%         derr = [derr;r.solution.phase(q).state(2:end,4)];
        taup = [taup;r.solution.phase(q).control(2:end,7)];
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fc = contact_forces(r.solution.phase(q),PAR,2);
		Fx3 = [Fx3;Fc(2:end,1)];
		Fy3 = [Fy3;Fc(2:end,2)];
        x = [x;(r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(end)];
        
        ddtheta = Fc(2:end,3);
        ddth = [ddth;ddtheta];
        ddveli = (1/Mi)*(r.solution.phase(q).control(2:end,7)/Pr-Ci*r.solution.phase(q).state(2:end,1));
        rderr = [rderr;ddveli/Rr-ddtheta];
        
    else
        theta = [theta;r.solution.phase(q).position(2:end,3)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end,3)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).position(2:end,1)];
        beta = [beta;r.solution.phase(q).position(2:end,2)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,3)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,4)];
        u = [u;r.solution.phase(q).control(2:end,5)];
        vp = [vp;r.solution.phase(q).control(2:end,6)];
        vn = [vn;r.solution.phase(q).control(2:end,7)];
        obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,3).^ind+r.solution.phase(q).control(2:end,4).^ind)];
        objC = [objC;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,5).^ind)];
%         objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        
        dalpha = [dalpha;r.solution.phase(q).velocity(2:end,1)];
        dbeta = [dbeta;r.solution.phase(q).velocity(2:end,2)];
        dveli = [dveli;r.solution.phase(q).state(2:end,1)];
        ierr = [ierr;r.solution.phase(q).state(2:end,2)];
%         imotor = [imotor;r.solution.phase(q).state(2:end,3)];
%         derr = [derr;r.solution.phase(q).state(2:end,4)];
% 		if size(r.solution.phase(q).control,2)>4
% 			taup = [taup;r.solution.phase(q).control(2:end,5)];
% 		else
			taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
% 		end
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fx3 = [Fx3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fy3 = [Fy3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
        x = [x;(r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end)];
        
        ddtheta = acceleration(r.solution.phase(q),PAR,2);
        ddth = [ddth;ddtheta];
        ddveli = (1/Mi)*(-Ci*r.solution.phase(q).state(2:end,1));
        rderr = [rderr;ddveli/Rr-ddtheta(2:end)];
    end
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.ddtheta = ddth;
out.taup = taup;
out.tau11 = tau11;
out.tau21 = tau21;
out.tau12 = tau12;
out.tau22 = tau22;
out.obj = obj;
objC(1) = trapz(out.time,u.^ind);
% objC(1) = sum(objC(2:end));
out.objC = objC;
out.dveli = dveli;
out.ierr = ierr;
out.imotor = u/Kmotor;
% out.derr = derr;
out.rderr = rderr;
out.err = dveli/Rr-dtheta;
% out.ddtheta = ddtheta;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.tau_s = out.tau11-out.tau12;
out.tau_e = out.tau21-out.tau22;
out.ctrlaw = Kte*(Jtotal*(dveli/Rr-dtheta)+Btotal*ierr);
out.ctrlawU = u;
out.ctrlawV = vp-vn;
out.x = x;

function out = closed_MBF_Observer(r,PAR)

nsim = numel(r.solution.phase);
theta = r.solution.phase(1).position(1);
dtheta = r.solution.phase(1).velocity(1);
time = r.solution.phase(1).time(1);
alpha = r.solution.phase(1).control(1,3);
beta = r.solution.phase(1).control(1,4);
tau11 = r.solution.phase(1).control(1,1);
tau12 = r.solution.phase(1).control(1,5);
tau21 = r.solution.phase(1).control(1,2);
tau22 = r.solution.phase(1).control(1,6);
ind = r.Options.Indice;
obj = r.solution.objective;
objC = 0;
dalpha = 0;
dbeta = 0;
% dveli = 0;
% ierr = 0;
imotor = 0;
% ddtheta = 0;
% dveli = r.solution.phase(1).state(1,1);
% ierr = r.solution.phase(1).state(1,2);
% imotor = r.solution.phase(1).state(1,3);
derr = r.solution.phase(1).state(1,1);
taup = r.solution.phase(1).control(1,7);
Fx3 = 0;
Fy3 = 0;
rderr = 0;
x = 0;
% CtrLaw = 0;

Mi = PAR.Mi;
Ci = PAR.Ci;
Fri = PAR.Fri;
Kte = PAR.const;
Jtotal = PAR.Jmt;
Btotal = PAR.Bt;
Kmotor = PAR.rt*PAR.Kt;
Ke = PAR.rt*PAR.Ke;
Ra = PAR.Ra;
La = PAR.La;
Td = PAR.Td;
Pr = PAR.R;
Rr = PAR.Rr;
Jeq = Mi*PAR.Rr^2;
Beq = Ci*PAR.Rr^2;
Trr = Fri*PAR.Rr;

for q=1:nsim
    
    if rem(q,2)
        theta = [theta;r.solution.phase(q).position(2:end)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).control(2:end,3)];
        beta = [beta;r.solution.phase(q).control(2:end,4)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,5)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,6)];
		if q == 1
			obj = [obj;trapz(r.solution.phase(q).time,r.solution.phase(q).control(:,1).^ind+r.solution.phase(q).control(:,2).^ind+r.solution.phase(q).control(:,5).^ind+r.solution.phase(q).control(:,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time,((1/Td)*Jeq*(r.solution.phase(q).velocity-r.solution.phase(q).state(:,1))+Beq*r.solution.phase(q).velocity+Trr).^ind)];
%             ((1/Td)*Jeq*(r.solution.phase(q).velocity-r.solution.phase(q).state(:,1))+Beq*r.solution.phase(q).velocity+Trr)/Kmotor
%             objC = [objC;trapz(r.solution.phase(q).time,Kmotor*r.solution.phase(q).state(:,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
		else
			obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,5).^ind+r.solution.phase(q).control(2:end,6).^ind)];
            objC = [objC;trapz(r.solution.phase(q).time(2:end),(1/Td)*Jeq*(r.solution.phase(q).velocity(2:end)-r.solution.phase(q).state(2:end,1))+Beq*r.solution.phase(q).velocity(2:end+Trr).^ind)];
%             objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        end
		
        [dalpha1,dbeta1] = idof_transform(vect(r.solution.phase(q).control(2:end,3)),vect(r.solution.phase(q).control(2:end,4)),vect(r.solution.phase(q).position(2:end)),vect(r.solution.phase(q).velocity(2:end)),PAR);
        dalpha = [dalpha;double(dalpha1)];
        dbeta = [dbeta;double(dbeta1)];
%         dveli = [dveli;r.solution.phase(q).state(2:end,1)];
%         ierr = [ierr;r.solution.phase(q).state(2:end,2)];
%         imotor = [imotor;r.solution.phase(q).state(2:end,1)/Kmotor];
        derr = [derr;r.solution.phase(q).state(2:end,1)];
        taup = [taup;r.solution.phase(q).control(2:end,7)];
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fc = contact_forces(r.solution.phase(q),PAR,3);
		Fx3 = [Fx3;Fc(2:end,1)];
		Fy3 = [Fy3;Fc(2:end,2)];
        x = [x;(r.solution.phase(q).position(2:end,1)-r.solution.phase(q).position(1,1))*PAR.Rr + x(end)];
        
        ddtheta = Fc(2:end,3);
%         ddveli = (1/Mi)*(r.solution.phase(q).control(2:end,7)/Pr-Ci*r.solution.phase(q).state(2:end,1));
%         rderr = [rderr;ddveli/Rr-ddtheta];
        
    else
        theta = [theta;r.solution.phase(q).position(2:end,3)];
        dtheta = [dtheta;r.solution.phase(q).velocity(2:end,3)];
        time = [time;r.solution.phase(q).time(2:end)];
        alpha = [alpha;r.solution.phase(q).position(2:end,1)];
        beta = [beta;r.solution.phase(q).position(2:end,2)];
        tau11 = [tau11;r.solution.phase(q).control(2:end,1)];
        tau21 = [tau21;r.solution.phase(q).control(2:end,2)];
        tau12 = [tau12;r.solution.phase(q).control(2:end,3)];
        tau22 = [tau22;r.solution.phase(q).control(2:end,4)];
        obj = [obj;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).control(2:end,1).^ind+r.solution.phase(q).control(2:end,2).^ind+r.solution.phase(q).control(2:end,3).^ind+r.solution.phase(q).control(2:end,4).^ind)];
        objC = [objC;trapz(r.solution.phase(q).time(2:end),r.solution.phase(q).state(2:end,1).^ind)];
%         objC = [objC;trapz(r.solution.phase(q).time(2:end),Kmotor*r.solution.phase(q).state(2:end,3).^ind)/(r.solution.phase(q).time(end)-r.solution.phase(q).time(1))];
        
        dalpha = [dalpha;r.solution.phase(q).velocity(2:end,1)];
        dbeta = [dbeta;r.solution.phase(q).velocity(2:end,2)];
%         dveli = [dveli;r.solution.phase(q).state(2:end,1)];
%         ierr = [ierr;r.solution.phase(q).state(2:end,2)];
%         imotor = [imotor;r.solution.phase(q).state(2:end,1)/Kmotor];
        derr = [derr;r.solution.phase(q).state(2:end,1)];
		if size(r.solution.phase(q).control,2)>4
			taup = [taup;r.solution.phase(q).control(2:end,5)];
		else
			taup = [taup;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		end
		% ddtheta = [ddtheta;r.solution.phase(q).state(2:end,5)];
		Fx3 = [Fx3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
		Fy3 = [Fy3;zeros(numel(r.solution.phase(q).time(2:end)),1)];
        x = [x;(r.solution.phase(q).position(2:end,3)-r.solution.phase(q).position(1,3))*PAR.Rr + x(end)];
        
        ddtheta = acceleration(r.solution.phase(q),PAR,3);
%         ddveli = (1/Mi)*(-Ci*r.solution.phase(q).state(2:end,1));
%         rderr = [rderr;ddveli/Rr-ddtheta(2:end)];
    end
end

out.time = time;
out.alpha = alpha;
out.beta = beta;
out.theta = theta;
out.dalpha = dalpha;
out.dbeta = dbeta;
out.dtheta = dtheta;
out.taup = taup;
out.tau11 = tau11;
out.tau21 = tau21;
out.tau12 = tau12;
out.tau22 = tau22;
out.imotor = ((1/Td)*Jeq*(dtheta-derr)+Beq*dtheta+Trr)/Kmotor;
out.obj = obj;
objC(1) = trapz(out.time,Kmotor*out.imotor.^ind);
% objC(1) = sum(objC(2:end));
out.objC = objC;
% out.dveli = dveli;
% out.ierr = ierr;
out.derr = derr;
% out.rderr = rderr;
% out.err = dveli/Rr-dtheta;
% out.ddtheta = ddtheta;
out.Fx3 = Fx3;
out.Fy3 = Fy3;
out.tau_s = out.tau11-out.tau12;
out.tau_e = out.tau21-out.tau22;
% out.ctrlaw = Kte*((Jtotal*Ra+La*Btotal)*(dveli/Rr-dtheta)+(Btotal*Ra+Ke)*ierr+La*Jtotal*(1/PAR.Td)*((dveli/Rr-dtheta)-derr))/Kmotor;
out.x = x;

function Fc = contact_forces_r(PAR,time,q,dq,tau_e,tau_s,tau_m,flag)

ap = q(:,1);
bt = q(:,2);
th = q(:,3);
q = vect(q)';

dap = dq(:,1);
dbt = dq(:,2);
dth = dq(:,3);
dq = vect(dq)';

if size(tau_e,2) > 1
    a_sf = tau_s(:,1);
    a_se = tau_s(:,2);
    a_ef = tau_e(:,1);
    a_ee = tau_e(:,2);
    [ap1, bt1] = dof_conversion(q(1),q(2),'Type', 'angle');
    [dap1, dbt1] = dof_conversion(dq(1),dq(2),'Type', 'angvel');
    T = MTG_smooth(vect(a_sf),vect(a_se),vect(a_ef),vect(a_ee),ap1,bt1,dap1,dbt1);
    tau_s = -(T(1)+T(2));
    tau_e = -(T(3)+T(4));
else
    tau_s = vect(tau_s);
    tau_e = vect(tau_e);
end

if numel(tau_m) == 0
    tau_m = zeros(numel(ap),1);
end

[~,~,PHI,dPHI] = idof_transform(vect(ap),vect(bt),vect(th),vect(dth),PAR);

if flag == 1
    [M1,ke1,G1,H1,k1] = four_bar_system(q, dq,PAR,'Type', 'Control');
else
    [M1,ke1,G1,H1,k1] = four_bar_system(q, dq,PAR,'Type', 'Reference');
end

%%
ddth = gradient(dth)./gradient(time);
ddth = interp1(time(1:end-2),ddth(1:end-2),time,'spline','extrap');
ddq = dPHI*vect(dth)+PHI*vect(ddth);
GF = M1(1:2,:)*ddq-(H1(1:2,:)*[tau_s;tau_e]+ke1(1:2)-k1(1:2));
Fci = G1(1:2,:)\GF;
Fci = double([Fci',vect(ddth)]);

%%
K = PHI'*M1*dPHI*vect(dth);
KE = PHI'*(H1*[tau_s;tau_e]+ke1-k1)-K+vect(tau_m);
MM = PHI'*M1*PHI;
ddt = MM\KE;
ddq = dPHI*vect(dth)+PHI*ddt;
rhs = double((M1*ddq-(H1*[tau_s;tau_e]+ke1-k1+[0;0;vect(tau_m)]))');
ddtheta = double(ddq(3));
MassMatrix = double(MM)/PAR.Rr/PAR.R;

for i=1:size(rhs,1)
    aux1 = double(col(G1,i));
    aux2 = aux1\rhs(i,:)';
    Fc(i,:) = [aux2',ddtheta(i),MassMatrix(i)];
end

Fc;
% Fc = Fci;
   
function Mm = equivalent_mass(alpha,beta,theta,dalpha,dbeta,dtheta,PAR)

nsize = numel(alpha);
alpha = vect(alpha);
beta = vect(beta);
theta = vect(theta);
q = [alpha;beta;theta];

dalpha = vect(dalpha);
dbeta = vect(dbeta);
dtheta = vect(dtheta);
dq = [dalpha;dbeta;dtheta];

M1 = four_bar_system(q,dq,PAR,'Type','Reference');
for i=1:nsize
    iM1 = inv(double(col(M1,i)));
    Jmt = 1/iM1(3,3);
    Mm(i,1) = Jmt/PAR.Rr/PAR.R;
end


function Fc = contact_forces(r,PAR,flag)

tau1 = vect(r.control(:,1));
tau2 = vect(r.control(:,2));
alpha = vect(r.control(:,3));
beta = vect(r.control(:,4));
if size(r.control,2) > 4
	tau1 = vect(r.control(:,1)-r.control(:,5));
	tau2 = vect(r.control(:,2)-r.control(:,6));
end
theta = vect(r.position(:,1));
dtheta = vect(r.velocity(:,1));

[dalpha,dbeta,PHI,dPHI] = idof_transform(alpha,beta,theta,dtheta,PAR);
q = [alpha;beta;theta];
qd = [dalpha;dbeta;dtheta];

if flag==0
    [M1,ke1,G1,H1,k1] = four_bar_system(q, qd,PAR, 'Type', 'Reference');
    K = PHI'*M1*dPHI*dtheta;
	KE = PHI'*(H1*[tau1;tau2]+ke1-k1)-K;
	MM = PHI'*M1*PHI;
	ddt = MM\KE;
	ddq = dPHI*dtheta+PHI*ddt;
	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1))');
    ddtheta = double(ddq(3));
elseif flag==1
    [M1,ke1,G1,H1,k1] = four_bar_system(q, qd,PAR,'Type', 'Control');
    K = PHI'*M1*dPHI*dtheta;
	imotor = vect(r.state(:,3));
	KE = PHI'*(H1*[tau1;tau2]+ke1-k1)-K+PAR.Kt*imotor;
	MM = PHI'*M1*PHI;
	ddt = MM\KE;
	ddq = dPHI*dtheta+PHI*ddt;
	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1+[0;0;PAR.Kt*imotor]))');
    ddtheta = double(ddq(3));
elseif flag==2
    [M1,ke1,G1,H1,k1] = four_bar_system(q, qd,PAR,'Type', 'Reference');
    K = PHI'*M1*dPHI*dtheta;
	tmotor = vect(r.control(:,8));
	KE = PHI'*(H1*[tau1;tau2]+ke1-k1)-K+tmotor;
	MM = PHI'*M1*PHI;
	ddt = MM\KE;
	ddq = dPHI*dtheta+PHI*ddt;
	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1+[0;0;tmotor]))');
    ddtheta = double(ddq(3));
elseif flag==3
    [M1,ke1,G1,H1,k1] = four_bar_system(q, qd,PAR,'Type', 'Reference');
    l_c = vect(PAR.const*(PAR.Jmf*(r.state(:,1)/PAR.Rr-r.velocity(:,end))+PAR.Bt*r.state(:,2)));
    K = PHI'*M1*dPHI*dtheta;
	KE = PHI'*(H1*[tau1;tau2]+ke1-k1)-K+l_c;
	MM = PHI'*M1*PHI;
	ddt = MM\KE;
	ddq = dPHI*dtheta+PHI*ddt;
	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1+[0;0;l_c]))');
    ddtheta = double(ddq(3));
else
    [M1,ke1,G1,H1,k1] = four_bar_system(q, qd,PAR,'Type', 'Control');
    K = PHI'*M1*dPHI*dtheta;
	imotor = vect(r.state(:,1));
	KE = PHI'*(H1*[tau1;tau2]+ke1-k1)-K+imotor;
	MM = PHI'*M1*PHI;
	ddt = MM\KE;
	ddq = dPHI*dtheta+PHI*ddt;
	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1+[0;0;imotor]))');
    ddtheta = double(ddq(3));
end

for i=1:size(rhs,1)
    aux1 = double(col(G1,i));
    aux2 = aux1\rhs(i,:)';
    Fc(i,:) = [aux2',ddtheta(i)];
end

function ddtheta = acceleration(r,PAR,flag)

tau1 = vect(r.control(:,1));
tau2 = vect(r.control(:,2));
% alpha = vect(r.control(:,3));
% beta = vect(r.control(:,4));
if size(r.control,2) > 2
	tau1 = vect(r.control(:,1)-r.control(:,3));
	tau2 = vect(r.control(:,2)-r.control(:,4));
end

alpha = vect(r.position(:,1));
beta = vect(r.position(:,2));
theta = vect(r.position(:,3));

dalpha = vect(r.velocity(:,1));
dbeta  = vect(r.velocity(:,2));
dtheta = vect(r.velocity(:,3));

q = [alpha;beta;theta];
qd = [dalpha;dbeta;dtheta];
[M1,ke1,~,H1,k1] = four_bar_system(q, qd,PAR);

if flag==0
	KE = H1*[tau1;tau2]+ke1-k1;
% 	MM = PHI'*M1*PHI;
	ddq = M1\KE;
% 	ddq = dPHI*dtheta+PHI*ddt;
% 	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1))');
    ddtheta = double(ddq(3));
elseif flag==1
	imotor = vect(r.state(:,3));
	KE = H1*[tau1;tau2]+ke1-k1+[0;0;PAR.Kt*imotor];
% 	MM = PHI'*M1*PHI;
	ddq = M1\KE;
% 	ddq = dPHI*dtheta+PHI*ddt;
% 	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1+[0;0;PAR.Kt*imotor]))');
    ddtheta = double(ddq(3));
elseif flag==2
	tmotor = vect(r.control(:,5));
	KE = H1*[tau1;tau2]+ke1-k1+[0;0;tmotor];
% 	MM = PHI'*M1*PHI;
	ddq = M1\KE;
% 	ddq = dPHI*dtheta+PHI*ddt;
% 	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1+[0;0;PAR.Kt*imotor]))');
    ddtheta = double(ddq(3));
else
	imotor = vect(r.state(:,1));
	KE = H1*[tau1;tau2]+ke1-k1+[0;0;imotor];
% 	MM = PHI'*M1*PHI;
	ddq = M1\KE;
% 	ddq = dPHI*dtheta+PHI*ddt;
% 	rhs = double((M1*ddq-(H1*[tau1;tau2]+ke1-k1+[0;0;PAR.Kt*imotor]))');
    ddtheta = double(ddq(3));
end