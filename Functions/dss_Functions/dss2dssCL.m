% Log: 
%   March 20, 16:00: changed output from [Acl,Bcl,Ccl,Dcl,Ecl] 
%                       to [SYS_CL_DSS, Acl, Bcl, Ccl, Dcl, Ecl];
%       put nonzeroD: 1 if you want a nonzero D
%       put nonzeroD: 0 if you want a    zero D
%       put nonzeroD: -1 if you want to keep it as it is from ss

function [SYS_CL_DSS, Acl, Bcl, Ccl, Dcl, Ecl] = dss2dssCL(P,C, nonzeroD);

if nargin == 2
    nonzeroD = 1;
end
if nonzeroD == 1
    [Ap Bp Cp Dp Ep] = ss2dssD(P);
    [Ac Bc Cc Dc Ec] = ss2dssD(C);
elseif nonzeroD == 0
    [Ap Bp Cp Dp Ep] = ss2dssD(P,0);
    [Ac Bc Cc Dc Ec] = ss2dssD(C,0);
elseif nonzeroD == -1
    [Ap Bp Cp Dp Ep] = dssdata(P);
    [Ac Bc Cc Dc Ec] = dssdata(C);
else
    disp('error. D must be one of the following {-1,0,1}');
    return;
end


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

SYS_CL_DSS = dss(Acl, Bcl, Ccl, Dcl, Ecl);

end