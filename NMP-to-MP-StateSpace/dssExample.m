%%% This script is to control (or see the behaviour) of Descriptor State
%%% Space (DSS) systems
% We'll consider (w.l.o.g) plants with nonzero D matrix  
%(since it's possible to convert any system (including SS) 
%with zero D matrices into DSS with nonzero matrix)
clear all;
close all;

format long;
home;


%Consider the following TF (plant)
TFs
Gp3 = tf(2*[1 1],[1 -1 1]);


P = ss(Gp3);


% Ap = P.A;
% Bp = P.B;
% Cp = P.C;
% Dp = P.D;
% Ep = eye(size(Ap));
[Ap Bp Cp Dp Ep] = ss2dssD(P);  %convert plant to dss with nonzero D matrix

cNum = [1 0];
cDen = 1;

cNum = num1;
cDen = den1;

c = tf(cNum, cDen);


gcl2 = feedback(Gp3,c);


[Ac Bc Cc Dc Ec] = tf2dss(cNum, cDen); % This function converts tf to dss
C = dss(Ac,Bc,Cc,Dc,Ec);




[Acl, Bcl, Ccl, Dcl, Ecl] = dss2dssCL(P,C);
Gcl = dss(Acl,Bcl,Ccl,Dcl,Ecl);

gcl1 = tf(Gcl);


if abs(norm(gcl1-gcl2)) > 0.1    
    (gcl1)    
    (gcl2)
else
    fprintf("norm(gcl1-gcl2) = %.13u\n",norm(gcl1-gcl2));
end



