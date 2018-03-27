function [c, ceq] = nonlcon(x,denP,degQ,S)
%% denominator constraints
sizeS = size(S);
m = sizeS(1);
n = sizeS(2);
DOF = max(0,n-m);

denGbar =  conv(denP, x(1:degQ+1)');
while denGbar(1) == 0
    denGbar = denGbar(2:end);
end

c = rhStabilityCriterion(denGbar);

position = 0; 
overcon = length(c)-DOF; %number of elements over the constraint
%which element to use as a constraint 
%(since we have an overconstrained system)
%start from position 0
cDen = c(position+1 : position + overcon + DOF);
% c=x(1:2);
% c=c*ones(length(c),1);
% c = [];

%% numerator constraints
sizeX = size(x);
if sizeS(2) == sizeX(1);
 num = S*x;
else
    num = 1;
    fprintf('sizeS(2): %i, sizeX(1): %i\n',sizeS(2),sizeX(1));
    error('sizeS(2) != sizeX(1)');    
    
end

cNum = rhStabilityCriterion(num');
cDen = -1;
c = [cNum; cDen];
% tol = 0.0;
% c = c + tol*ones(length(c),1);


ceq = [];