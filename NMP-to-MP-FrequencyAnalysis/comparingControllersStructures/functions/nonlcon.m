function [c, ceq] = nonlcon(x,denP,degQ)

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
c = rhStabilityCriterion(denGbar);
% c=x(1:2);
% c=c*ones(length(c),1);
ceq = [];