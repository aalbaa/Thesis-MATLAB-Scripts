    function [Acl, Bcl, Ccl, Dcl, Ecl] = dss2dssCL(P,C);

[Ap Bp Cp Dp Ep] = ss2dssD(P);
[Ac Bc Cc Dc Ec] = ss2dssD(C);

% 
% Ap = P.A;
% Bp = P.B;
% Cp = P.C;
% Dp = P.D;
% Ep = P.E;
% 
% 
% 
% Ac = C.A;
% Bc = C.B;
% Cc = C.C;
% Dc = C.D;
% Ec = C.E;
% 


Dk = inv(1+Dc*Dp);
% Cclp = Cp-Dp*Dk*Dp;   % Mistake
Cclp = Cp-Dp*Dk*Cp;     % Mistake corrected 
Cclc = -Dp*Dk*Cc;

Ccp = Dk*Dc*Cp;
Ccc = Dk*Cc;
Dcc = Dk*Dc*Dp;


Dcl = Dp-Dp*Dk*Dc*Dp;

Ccl = [Cclp,    Cclc];

Acl = [Ap-Bp*Ccp,  -Bp*Ccc;
        Bc*Cclp,   Ac+Bc*Cclc];

Bcl = [Bp*(1-Dcc);
        Bc*Dcl];
    
Ecl = blkdiag(Ep,Ec);
