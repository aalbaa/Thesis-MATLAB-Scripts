
function [SYS_CL_DSS, Acl, Bcl, Ccl, Dcl] = ss2ssCL(Psys,Csys);

[Ap Bp Cp Dp] = ssdata(Psys);
[Ac Bc Cc Dc] = ssdata(Csys);



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

SYS_CL_DSS = ss(Acl, Bcl, Ccl, Dcl);

end