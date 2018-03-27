function [c] = nonlconLMI(x, numPlant,denPlant,degP,degQ)
degNumP  = length(numPlant)-1;
degDenP = length(denPlant)-1;

q = x(1:degQ+1);
p = x(degQ+2:degQ+2+degP);

% convolution of numerator
num1 = conv(numPlant,q);
num2 = conv(denPlant,p);

GbarDen = conv(denPlant,q);

% cDen = rhStabilityCriterion(GbarDen');

% roots

% cNum = real(roots(GbarNum'));
cDen = real(roots(GbarDen'));
% c = cNum;
c = cDen;




%% constraints specifically for Plant 3 (Gp3) and controller of degree 1/1
% q0 = x(2);
% q1 = x(1);
% 
% tol = 0.001;
% c = [-q0;
%     -q1;
%     10*q1-q0;
%     10*q0-100*q1;
%     100*q0*q1-(q0-10*q1)*(100*q1-10*q0);];
% c = c + tol*ones(length(c),1);
% 
% ceq = [];
%     
