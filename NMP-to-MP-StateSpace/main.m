% Amro Al Bali
% February 6, 2018
%NMP to MP plant in state space using PFF
%The plant used is AS
clear all;
close all;
home;

P = tf([1 -2],[1 10 100],1);

[Ap Bp Cp Dp] = tf2ss(P.Numerator{1,1},P.Denominator{1,1});




eigDes = [-1 -2];

Kbar = place(Ap,Bp,eigDes);

K = pinv(Cp')*Kbar';

Acl = Ap-Bp*K*Cp;
Bcl = -Bp*K*Dp;
Ccl = inv(1+Dp*K);
Dcl = 0;
