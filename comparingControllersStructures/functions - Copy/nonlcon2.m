function [c, ceq] = nonlcon2(x,Gp,degP,degQ);
numPlant = Gp.numerator{1,1};
denPlant = Gp.denominator{1,1};
degNumP  = length(numPlant-1);
degDenP = length(denPlant-1);

q = x(1:degQ+1);
p = x(degQ+2:degQ+2+degP);

% convolution of numerator
num1 = conv(numPlant,q);
num2 = conv(denPlant,p);

diff = length(num1)-length(num2); %difference in degree
if diff<0 %i.e. deg(b*q)<deg(a*p)
    num1 = [zeros(1,-diff) num1];
else
    num2 = [zeros(1,diff) num2];
end

GbarNum = num1+num2;

GbarDen = conv(denPlant,q);

% % RH stability criterion
% cNum = rhStabilityCriterion(GbarNum');
% cDen = rhStabilityCriterion(GbarDen');
% 
% % roots
% 
% % cNum = real(roots(GbarNum'));
% cDen = real(roots(GbarDen'));
% % c = cNum;
% c = cDen;
% ceq = [];



%% constraints specifically for Plant 3 (Gp3) and controller of degree 1/1
q0 = x(2);
q1 = x(1);

tol = 0.001;
c = [-q0;
    -q1;
    10*q1-q0;
    10*q0-100*q1;
    100*q0*q1-(q0-10*q1)*(100*q1-10*q0);];
c = c + tol*ones(length(c),1);

ceq = [];
    
