function C = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
%% Given
% b=[1 -1]; %numerator
% a=[1 -2 0]; %denomenatr
% degP=1; %controller of controller's denomenatr
% degQ=1; %degree of controller's numerator
% c = [1 6 11 6]'; %desired poles
% c=desiredPoles;
c = poly(desiredPoles)';
%% operations
degB=length(numPlant);
degA=length(denPlant);
%degA>=degB
numPlant=[zeros(1,degA-degB) numPlant];
m=degP;
Tden = toeplitz([denPlant zeros(1,m)],[denPlant(1) zeros(1,m)]);
Tnum = toeplitz([numPlant zeros(1,m)],[numPlant(1) zeros(1,m)]);
% S=[Tnum Tden];
C=[Tnum Tden];
% sol=S\c;
%% sol
% C = tf(sol(1:degQ+1)',sol(degQ+2:degQ+2+degP)');