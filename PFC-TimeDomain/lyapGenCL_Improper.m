% This script uses the Generalized Lyapunov function A'PE+E'PA=-E'YE (Y>0,P>0) 
% to synthesize a feedback controller C that stabilizes the plant P.
% Some variables are assumed to be given for the ease of caluclations (or
% even to make it possible to solve the LMI) such as:
%   Assuming that Cc is given
%   Assuming that Dc is given (Dc!=0 to ensure it's bi-proper)
%   Assuming Ec is given

clear all;
close all;
% home;
eps = 1e-5;

Ptf = tf([1 -2],[1 10 100]);    

n = 2;

[Ap, Bp, Cp, Dp, Ep] = dssdata(Ptf);
Ep = blkdiag(Ep,zeros(n));
Ap = blkdiag(Ap,eye(n));
Bp = [Bp;ones(n,1)];
Cp = [Cp,zeros(1,n)];
    

np = size(Ap,1);
if rem(np,2) == 1
    Ep = blkdiag(Ep,0);
    Ap = blkdiag(Ap,1);
    Bp = [Bp;1];
    Cp = [Cp,0];
end
np = size(Ap,1);

nc = np; % I need P2 to be invertible (and square) to recover Ac and Bc

% P=[p1,p2; p2', p2];
P1 = sdpvar(np); % need it to be symmetric

nc1 = nc/2;

P21 = sdpvar(nc1);%,nc,'full');
P22 = P21;
P23 = sdpvar(nc1);

P2 = [P21, P21;
      P21, P23];
  
P3 = P2';
P4 = P2;

P = [P1, P2;
    P2', P2];

Y = sdpvar(nc+np);

% Acbar = sdpvar(nc,nc,'full');
% Bcbar = sdpvar(nc,1);
Ac1bar = sdpvar(nc1,nc1,'full');
Bc1bar = sdpvar(nc1,1,'full');

Acbar = [Ac1bar, P22;
        Ac1bar, P23];

B2 = zeros(nc1,1);B2(nc1)=-1;
Bcbar =[Bc1bar+P22*B2;
        Bc1bar+P23*B2];

    


Cc1 = ones(1,nc1); % assuming some arbitrary structure
Cc = [Cc1, -flip(B2)'];
Dc = 0;
% Ec = eye(nc);Ec(nc,nc)=0;Ec(2,1)=1;
E1 = eye(nc1);
E2 = zeros(nc1); 
for i=1:nc1-1
    E2(i,i+1)=1;
end

Ec = blkdiag(E1,E2);

% Ec = 10*rand(nc);
% 
PAcl = [P1*Ap-P1*Bp*Dc*Cp+Bcbar*Cp,     -P1*Bp*Cc+Acbar,
        P3*Ap-P3*Bp*Dc*Cp+Bcbar*Cp,     -P3*Bp*Cc+Acbar];

% PAcl = [P1*Ap-P1*Bp*Dc*Cp+Bcbar*Cp,     -P1*Bp*Cc+P2*Ac,
%         P3*Ap-P3*Bp*Dc*Cp+Bcbar*Cp,     -P3*Bp*Cc+P2*Ac];

E = blkdiag(Ep,Ec);
%%% LMIs:
LMI1 = [E'*PAcl+PAcl'*E == -E'*Y*E];

LMI2 = [P >= eps];
LMI3 = [Y >= eps];


%%% KYP LMI;

Q = sdpvar(np);
Q = P2;

KYP1 = [ [Acbar+Acbar',      Cc'-Bcbar;
         Cc-Bcbar',  -(Dc+Dc)] <= -eps];
% 
F = [LMI1, LMI2];



% F = [LMI1, LMI2, LMI3, KYP1];




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




Csys = dss(Ac,Bc,Cc,Dc,Ec);

Ctf = tf(Csys);

Gcl = feedback(Ptf,Ctf);

Gbar = Ptf+1/Ctf;