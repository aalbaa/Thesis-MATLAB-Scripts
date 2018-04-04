%%% This script solves for the 


% clear all;
% close all;
% home;
% 
% Ptf = tf([1 -2],[1 -10 100]);
% [Ap, Bp, Cp, Dp] = ssdata(Ptf);
% 
% np = size(Ap,1);

Q = sdpvar(np);
Ccbar = sdpvar(1,np);
Dc = sdpvar(1);

LMI1 = [ [Ac*Q+Q*Ac',      Ccbar'-Bc;
        Ccbar-Bc',  -(Dc+Dc)] <= -eps];
LMI2 = [ Q >= eps];


F = [LMI1, LMI2];




opt = sdpsettings;
opt.verbose = verb;
% opt.removeequalities = [1 (0)];


if solver == 1    
    opt.solver = 'mosek';
    opt.mosek.MSK_DPAR_ANA_SOL_INFEAS_TOL = 1e-3;
elseif solver == 2       
    opt.solver = 'sdpt3';
elseif solver == 3        
    opt.solver = 'sedumi';
else
    opt = [];
end


sol=solvesdp(F,[],opt);

Q = double(Q);
Ccbar = double(Ccbar);
% Acbar = double(Acbar);

Cc = ((inv(Q)*Ccbar')');
Dc = double(Dc);

Csys = ss(Ac,Bc,Cc,Dc);

Ctf = tf(Csys);

Gcl = feedback(Ptf,Ctf);

Gbar = Ptf+1/Ctf;

