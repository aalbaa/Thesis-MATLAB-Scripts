% This script uses the Generalized Lyapunov function A'PE+E'PA=-E'YE (Y>0,P>0) 
% to synthesize a feedback controller C that stabilizes the plant P.
% Some variables are assumed to be given for the ease of caluclations (or
% even to make it possible to solve the LMI) such as:
%   Assuming that Cc is given
%   Assuming that Dc is given (Dc!=0 to ensure it's bi-proper)
%   Assuming Ec is given

% clear all;
% close all;
% % home;

% function [Gbar, Cfb] = lyapGenCL(Psys, solver, verb, eps1)
%%% solver params
% solver = 1;
% verb = 1;
% eps = 1e-5;


% 
% if nargin <= 3
%     eps1 = 1e-5;
%     if nargin <= 2
%         verb = 0;
%         if nargin == 1
%             solver = 1;
%         end
%     end
% end

% Ptf = tf([1 -2],[1 10 100]);    

[Ap, Bp, Cp, Dp, Ep] = dssdata(Psys);

% Ep = blkdiag(Ep,0);
% Ap = blkdiag(Ap,1);
% Bp = [Bp;1];
% Cp = [Cp,0];

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

Cc = ones(1,nc); % assuming some arbitrary structure
Dc = 1;
Ec = eye(nc);
% Ec(nc,nc)=0;Ec(2,1)=1;
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

% Q = sdpvar(np);
Q = P2;

KYP1 = [ [Acbar+Acbar',      Cc'-Bcbar;
         Cc-Bcbar',  -(Dc+Dc)] <= -eps];
KYP2 = [P2*Ec == Ec'*P2];

KYP3 = [P2*Ec >= eps];
% 
% F = [LMI1, LMI2, KYP1];



F = [LMI1, LMI2, LMI3, KYP1, KYP2, KYP3];









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



success = ~isempty(findstr(sol.info,'Successfully'));

if ~success
    disp('Error: no solution or some other error');
    sol
    return;
end


%%% recovering matrices

P2 = double(P2);

P2inv = inv(P2);

Ac = P2inv*double(Acbar);
Bc = P2inv*double(Bcbar);

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

