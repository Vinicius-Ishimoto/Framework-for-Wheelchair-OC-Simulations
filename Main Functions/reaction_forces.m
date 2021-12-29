function Fr = reaction_forces(PAR,output,phases)

if nargin==2
    ncyc = output.Options.NumCycles;
    phases = 2*0:ncyc+1;
end


for i=phases
    cnt = 2*i+1;
    
    time = output.solution.phase(cnt).time;
    
    ap  = output.solution.phase(cnt).control(:,3);
    bt  = output.solution.phase(cnt).control(:,4);
    th  = output.solution.phase(cnt).position;
    dth = output.solution.phase(cnt).velocity;
    
    as_e  = output.solution.phase(cnt).state(:,1);
	as_f  = output.solution.phase(cnt).state(:,3);
	ae_e  = output.solution.phase(cnt).state(:,2);
	ae_f  = output.solution.phase(cnt).state(:,4);

    Fr = f_runner(time,ap,bt,th,dth,as_e,as_f,ae_e,ae_f,PAR);
end



function out = f_runner(time,ap,bt,th,dth,a_se,a_sf,a_ee,a_ef,PAR)

dap  = gradient(ap)./gradient(time);
dap = interp1(time(1:end-2),dap(1:end-2),time,'spline','extrap');
ddap = gradient(dap)./gradient(time);
ddap = interp1(time(1:end-2),ddap(1:end-2),time,'spline','extrap');

dbt  = gradient(bt)./gradient(time);
dbt = interp1(time(1:end-2),dbt(1:end-2),time,'spline','extrap');
ddbt = gradient(dbt)./gradient(time);
ddbt = interp1(time(1:end-2),ddbt(1:end-2),time,'spline','extrap');
% PP = spline(time,bt);
% p_der = fnder(PP,1);
% dbt = ppval(p_der,time);
% dbt = interp1(time(1:end-2),dbt(1:end-2),time,'spline','extrap');
% p_der = fnder(PP,2);
% ddbt = ppval(p_der,time);
% ddbt = interp1(time(1:end-2),ddbt(1:end-2),time,'spline','extrap');

ddth = gradient(dth)./gradient(time);
ddth = interp1(time(1:end-2),ddth(1:end-2),time,'spline','extrap');
% PP = spline(time,dth);
% p_der = fnder(PP,1);
% ddth = ppval(p_der,time);
% ddth = interp1(time(1:end-2),ddth(1:end-2),time,'spline','extrap');

q   = [vect(ap);vect(bt);vect(th)];
dq  = [vect(dap);vect(dbt);vect(dth)];
ddq = [vect(ddap);vect(ddbt);vect(ddth)];

[ap1, bt1] = dof_conversion(q(1),q(2),'Type', 'angle');
[dap1, dbt1] = dof_conversion(dq(1),dq(2),'Type', 'angvel');
T = MTG_smooth(vect(a_sf),vect(a_se),vect(a_ef),vect(a_ee),ap1,bt1,dap1,dbt1);
tau_s = -(T(1)+T(2));
tau_e = -(T(3)+T(4));
Tau = [tau_s;tau_e];

[M1,ke1,G1,H1,k1] = four_bar_system(q, dq,PAR,'Type','Reference');

GF = M1*ddq - (H1*Tau + ke1 - k1);
GFv = double(GF');
Tp = GF(3);

for i=1:size(GFv,1)
    aux1 = double(col(G1,i));
    aux2 = aux1\GFv(i,:)';
    out(i,:) = [aux2',double(col(Tp,i))];
end

%% Validation

[dap_v,dbt_v] = idof_transform(vect(ap),vect(bt),vect(th),vect(dth),PAR);
dap_v = double(dap_v);
dbt_v = double(dbt_v);

for i=1:size(GFv,1)
    aux1 = double(col(G1,i));
    Tp_v(i,1) = aux1(3,:)*out(i,1:2)';
end

out;