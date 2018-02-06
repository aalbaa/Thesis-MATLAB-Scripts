function [c, ceq] = nonlcon(x,denP,degQ,sizeS)
m = sizeS(1);
n = sizeS(2);
DOF = max(0,n-m);
% a = 1;
% b = 2;
% numP = [1 -b];
% denP = [1 a 0];
% Gp1 = tf(numP,denP);
% 
% degP = 1;
% degQ = 2;
denGbar =  conv(denP, x(1:degQ+1)');
while denGbar(1) == 0
    denGbar = denGbar(2:end);
end
tol = 0.1;
c = rhStabilityCriterion(denGbar);
c = c + tol*ones(length(c),1);
position = 0; 
overcon = length(c)-DOF; %number of elements over the constraint
%which element to use as a constraint 
%(since we have an overconstrained system)
%start from position 0
c = c(position+1:position + overcon + DOF);
% c=x(1:2);
% c=c*ones(length(c),1);
% c = [];

ceq = [];