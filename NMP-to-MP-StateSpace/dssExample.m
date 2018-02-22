%%% This script is to control (or see the behaviour) of Descriptor State
%%% Space (DSS) systems
% We'll consider (w.l.o.g) plants with nonzero D matrix  
%(since it's possible to convert any system (including SS) 
%with zero D matrices into DSS with nonzero matrix)
clear all;
close all;
home;


%Consider the following TF (plant)

p = tf(2*[1 1],[1 -1 1]);

P = ss(p);



[Ap Bp Cp Dp Ep] = ss2dssD(P);

c = tf([1 0],1);

C = ss(c);

% [Ac Bc Cc Dc Ec] = ss2dssD(C);


[Acl, Bcl, Ccl, Dcl, Ecl] = dss2dssCL(P,C);
Gcl = dss(Acl,Bcl,Ccl,Dcl,Ecl);

gcl = tf(Gcl);





