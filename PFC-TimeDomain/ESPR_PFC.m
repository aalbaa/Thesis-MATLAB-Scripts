% This script is dedicated to SYTHESIZING A PARALLEL-FEEDFORWARD CONTROLLER
clear all;
close all;

% params:
eps = 1e-8;
solver = 1;
verb = 1;


TFs;

% P = tf([1 -1],[1 1 1]);
P = Gp3;
Ptf = P;
% [Ap,Bp,Cp,Dp,Ep] = dssdata(P);
[Ap,Bp,Cp,Dp,Ep, Pdss] = ss2dssD(P,1);

np = size(Ap,1);

nc = np;  % design variable

Ecbar = sdpvar(nc,nc,'full');

% Ec = eye(nc);
% E = blkdiag(Ep,Ec);

Acbar = sdpvar(nc,nc,'full');
Bc = sdpvar(nc,1);
% Bc = [1;0];

Ccbar = sdpvar(1,nc);
Dc = sdpvar(1);
% Dc = 0;


B = [Bp;Bc];
D = Dp+Dc;

n = nc+np;


% LMI
%     Q1 = sdpvar(np,np,'full');
    Q1 = sdpvar(np);
%     Q2 = sdpvar(np,np,'full');
    Q2 = sdpvar(np);
    Q3 = Q2;
    Q4 = Q2;
    Q = [Q1, Q2; Q3 Q4];
        
    AQ = [Ap*Q1, Ap*Q2;
            Acbar,  Acbar];
    
%         AQt = [Ap*Q1+(Ap*Q1)', Ap*Q2+(Acbar)';
%             Acbar+(Ap*Q2)',  Acbar+(Acbar)']; % it's actually AQ+(AQ')

    CQ = [Cp*Q1+Ccbar, Cp*Q2+Ccbar];
      
    EQ = [Ep*Q1, Ep*Q2;
        Ecbar, Ecbar];
    
%     EQ = E*Q;
    
    M1 = [AQ+(AQ)',     B-(CQ)'
            B'-(CQ),       -(D+D')]; % < 0
    
    
    
    LMI1 = [M1 <= -eps];

    M2 = EQ; % >=0
    LMI2 = [M2 >= 0];

    t = [];
    M3 = EQ - (EQ)';
    LMI3 = [M3 == 0]; % equality constraint    
    
    F = [LMI1, LMI2, LMI3];
    
    


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

Q2 = double(Q2);
Ec = double(Ecbar)*inv(Q2);
Ac = double(Acbar)*inv(Q2);
Cc = double(Ccbar)*inv(Q2);
Bc = double(Bc);
Dc = double(Dc);

Cdss = dss(Ac,Bc,Cc,Dc,Ec);
Ctf = tf(Cdss);

Gpfc = Ptf+Ctf;

