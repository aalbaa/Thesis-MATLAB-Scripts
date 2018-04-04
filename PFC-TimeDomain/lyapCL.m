% This script uses the Lyapunov function A'P+PA<0 to synthesize a feedback
% controller C that stabilizes the plant P.
% Some variables are assumed to be given for the ease of caluclations (or
% even to make it possible to solve the LMI) such as:
%   Assuming that Cc is given
%   Assuming that Dc is given (Dc!=0 to ensure it's bi-proper)
% 
clear all;
close all;
% home;

Ptf = tf([1 -2],[1 10 100]);
[Ap, Bp, Cp, Dp] = ssdata(Ptf);

np = size(Ap,1);

nc = np; % I need P2 to be invertible (and square) to recover Ac and Bc

% P=[p1,p2; p2', p2];
P1 = sdpvar(np); % need it to be symmetric
P2 = sdpvar(np);%,nc,'full');
P3 = P2';
P4 = P2;

P = [P1, P2;
    P2', P2];

Acbar = sdpvar(nc,nc,'full');
Bcbar = sdpvar(nc,1);

Cc = ones(1,nc); % assuming some arbitrary structure
Dc = 1;
% 
PAcl = [P1*Ap-P1*Bp*Dc*Cp+Bcbar*Cp,     -P1*Bp*Cc+Acbar,
        P3*Ap-P3*Bp*Dc*Cp+Bcbar*Cp,     -P3*Bp*Cc+Acbar];

% PAcl = [P1*Ap-P1*Bp*Dc*Cp+Bcbar*Cp,     -P1*Bp*Cc+P2*Ac,
%         P3*Ap-P3*Bp*Dc*Cp+Bcbar*Cp,     -P3*Bp*Cc+P2*Ac];


%%% LMIs:
LMI1 = [PAcl+PAcl' <= -eps];

LMI2 = [P >= eps];


%%% KYP LMI;

% Q = sdpvar(np);
% Q = P2;
% 
% KYP1 = [ [Ac*Q+Ac'*Q,      Cc'-Bcbar;
%          Cc-Bcbar',  -(Dc+Dc)] <= -eps];
% 
% F = [LMI1, LMI2, KYP1];



F = [LMI1, LMI2];




%%% solver params
solver = 1;
verb = 1;
eps = 1e-5;



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



%%% recovering matrices

P2 = double(P2);

P2inv = inv(P2);

Ac = P2inv*double(Acbar);
Bc = P2inv*double(Bcbar);

Csys = ss(Ac,Bc,Cc,Dc);

Ctf = tf(Csys);

Gcl = feedback(Ptf,Ctf);

Gbar = Ptf+1/Ctf;