% This script is to test whether finding a solution to ``A+A'< 0 " imply
% `` A< 0"

% Let's start with a simple example

clear all;
close all;
home;

Ptf = tf(1,[1 -1 1]);
% [~,Ctf]=polePlacement(Ptf,2,1);

Ptf = tf([1 1 1],[1 1]); 
[Pdss,Ap, Bp, Cp, Dp, Ep] = tf2dss(Ptf);


[S D V] = svd(Ep);

n1 = rank(Ep);
n2 = size(Ep,1)-n1;

Tr = V'; % x = T*xnew

Tl = S';    %lower case L for 'left'
d = diag(D);

Tl2 = diag( [1./d(1:n1); ones(n2,1)]); % lower case L for 'left'

Enew = Tl2*Tl*Ep*Tr;

Anew = Tl2*Tl*Ap*Tr;

Bnew = Tl2*Tl*Bp;

Cnew = Cp*Tr;



A = Anew(1:n1,1:end);







% Gcl = feedback(Ptf,Ctf);

% [Ap, Bp, Cp, Dp, Ep] = dssdata(Ptf);
% 
% lam = sdpvar(1);
% 
% M = [lam*Ep,   Ap;
%     Ap',    lam*Ep];
% 
% t = lam^2;
% 
% F = [M >= 0];
% 
% solver = 1;
% verb = 1;
% 
% opt = sdpsettings;
% opt.verbose = verb;
% opt.removeequalities = 0;
% 
% if solver == 1    
%     opt.solver = 'mosek';
%     opt.mosek.MSK_DPAR_ANA_SOL_INFEAS_TOL = 1e-3;
% elseif solver == 2       
%     opt.solver = 'sdpt3';
% %     opt.sedumi.numtol = 1e-4;
% %     opt.sedumi.eps = 1e-5;
% %     opt.sedumi.bigeps = 1e-2;
% %     opt.sedumi.bignumtol = 1;
% elseif solver == 3        
%     opt.solver = 'sedumi';
% else
%     opt = [];
% end
% 
% 
% sol=solvesdp(F,t,opt)

