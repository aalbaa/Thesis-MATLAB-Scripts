% This script uses the Generalized Lyapunov function A'PE+E'PA=-E'YE (Y>0,P>0) 
% to synthesize a feedback controller C that stabilizes the plant P.
% Some variables are assumed to be given for the ease of caluclations (or
% even to make it possible to solve the LMI) such as:
%   Assuming that Cc is given
%   Assuming that Dc is given (Dc!=0 to ensure it's bi-proper)
%   Assuming Ec is given

% clear all;
% close all;
% home;

% function [Gbar, Cfb] = lyapGenCL(Psys, solver, verb, eps1)
%% solver params
solver = 1;
verb = 1;
eps1 = 1e-5;

% 
% 
% if nargin <= 3
%     eps1 = 1e-5;
%     if nargin <= 2
%         verb = 0;
%         if nargin <= 1
%             solver = 1;                 
%         end
%     end
% end

Psys = tf([1 -2],[1 10 100]);    

[Ap, Bp, Cp, Dp, Ep] = dssdata(Psys);



nc = 3;

np = size(Ap,1);

Ep = blkdiag(Ep,zeros(nc-np));
Ap = blkdiag(Ap,eye(nc-np));
Bp = [Bp;ones(nc-np,1)];
Cp = [Cp,zeros(1,nc-np)];

np = size(Ap,1);


nc = np; % I need P2 to be invertible (and square) to recover Ac and Bc

% P=[p1,p2; p2', p2];
P1 = sdpvar(np); % need it to be symmetric
P2 = sdpvar(np);%,nc,'full');
P3 = P2';
P4 = P2;

P = [P1, P2;
    P2', P2];

Y = sdpvar(nc+np);

Acbar = sdpvar(nc,nc,'full');
Bcbar = sdpvar(nc,1);
% Ecbar = sdpvar(nc,nc,'full');

% Cc = [1 zeros(1,nc-1)]; % assuming some arbitrary structure
% Cc = ones(1,nc);
Cc = place(Ap,Bp,-1:-1:-nc);
% Cc = [-3.4500  -14.4663  -19.1320];
% Cc = [0 1];
Dc = 1;
Ec = eye(nc);
Ec(nc,nc)=0;

% Ec =[0    0.0625         0
%          0         0    0.1250
%          0         0    1.0000];

% Ec = [1, 1;0 1];
% Ec(nc,nc)=0;Ec(2,1)=1;
% Ec = 10*rand(nc);
% 
PAcl = [P1*Ap-P1*Bp*Dc*Cp+Bcbar*Cp,     -P1*Bp*Cc+Acbar,
        P3*Ap-P3*Bp*Dc*Cp+Bcbar*Cp,     -P3*Bp*Cc+Acbar];

% PAcl = [P1*Ap-P1*Bp*Dc*Cp+Bcbar*Cp,     -P1*Bp*Cc+P2*Ac,
%         P3*Ap-P3*Bp*Dc*Cp+Bcbar*Cp,     -P3*Bp*Cc+P2*Ac];

E = blkdiag(Ep,Ec);
% PE = [P1*Ep,    Ecbar;
%       P2*Ep,    Ecbar];
  
%%% LMIs:
LMI1 = [E'*PAcl+PAcl'*E == -E'*Y*E];

LMI2 = [P >= eps1];
LMI3 = [Y >= eps1];


%%% KYP LMI;

% Q = sdpvar(np);
Q = P2;

KYP1 = [ [Acbar+Acbar',      Cc'-Bcbar;
         Cc-Bcbar',  -(Dc+Dc)] <= -eps];
KYP2 = [P2*Ec == Ec'*P2];

KYP3 = [P2*Ec >= eps];

% F = [LMI1, LMI2, LMI3, KYP1];

%%% minimum gain lemma:

Dcinv = inv(Dc);

Cff = -Dcinv*Cc;
Dff = Dcinv;

nu = sdpvar(1);

% LMI4 = [[Acbar+Acbar'-Bcbar*Dcinv*Cc-Cc'*Dcinv*Bcbar',  Bcbar*Dcinv-Cff'*Dff;
%           Dcinv'*Bcbar'-Dff'*Cff,        nu-Dff'*Dff] <= 0];

LMI4 = [[Acbar+Acbar'-Cc'*Cc,  Bcbar- Cc'*Dc , zeros(nc,1);
          Bcbar'-Dc'*Dc,        -Dc'*Dc,     nu;
          zeros(1,nc),  nu',    -1] <= 0];


LMI5 = [P2*Ec==Ec'*P2'];

LMI6 = [P2*Ec >= 0];
LMI7 = [nu >= 0];



% GLE on ff
PAff = Acbar-Bcbar*Cc/Dc;
P2 = sdpvar(size(Acbar,1));
Y = sdpvar(size(Acbar,1));

LMI8 = [P2 >= eps1];
LMI9 = [Y >= eps1];
LMI10 = [Ec'*Acbar+Acbar'*Ec-(Ec'*Bcbar*Cc/Dc+Cc'*Bcbar'*Ec/Dc) == - Ec*Y*Ec];



% F = [LMI1, LMI2, LMI3, LMI4, LMI5, LMI6, LMI7];
% F = [LMI4, LMI5, LMI6, LMI7];
% F = [LMI8, LMI9, LMI10];
% F = [LMI1, LMI2, LMI3, LMI9, LMI10];
F = [LMI1, LMI2, LMI3, LMI4, LMI5, LMI6];
% F = [LMI4];

t = (-nu);
% t = [];



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


% sol=solvesdp(F,[],opt);
sol=solvesdp(F,t,opt);




success = ~isempty(findstr(sol.info,'Successfully'));

if ~success
    disp('Error: no solution or some other error');
    sol
    return;
end


%%% recovering matrices

P2 = double(P2);
P2 = double(P3);

P2inv = inv(P2);

Ac = P2inv*double(Acbar);
Bc = P2inv*double(Bcbar);

Aff = Ac-Bc*Dcinv*Cc;
Bff = Bc*Dcinv;

Gff = tf(dss(Aff,Bff,Cff,Dff,Ec));

Ctf1 = tf(dss(Ac,Bc,Cc,Dc,Ec));
% ni = 2; % improper
% Ac = blkdiag(Ac,eye(ni));
% if ni>0
%     B2 = zeros(ni,1);B2(ni)=-1;
% end
% Bc = [Bc;B2];
% Cc = [Cc,-flip(B2)'];
% E2 =zeros(ni);
% for i=1:ni-1
%     E2(i,i+1)=1;
% end
% Ec = blkdiag(Ec,E2);

Csys = dss(Ac,Bc,Cc,Dc,Ec);

Ctf = tf(Csys);

Gcl = feedback(Psys,Ctf);

Gbar = Psys+1/Ctf;

% 
% bode(Psys,Gbar);
% legInfo{1} = ['$P(s)$'];
% legInfo{2} = ['$\bar{G}(s)$'];
% leg = legend(legInfo);
% leg.Interpreter = 'latex';
% leg.FontSize = 12;


Cfb = Ctf;
% end

