% attempt to find a solution to underdetermined system
% clear all;
function [Gbar C] = infManySols(Gplant,degP,degQ,desiredPoles)
% numPlant = Gplant.Numerator{1,1};
% denPlant = Gplant.Denominator{1,1};
%% Inputs
numPlant = Gplant.Numerator{1,1};
denPlant = Gplant.Denominator{1,1};
%removing leading zeros (Gplant.Numerator/Denomenator returns the vector
%with leading zeros)

numPlant = numPlant(find(numPlant>0,1):end);
denPlant = denPlant(find(denPlant>0,1):end);

degB=length(numPlant)-1;
degA=length(denPlant)-1;
% degC = max(degA,degB)+max(degP,degQ);
degC = max(degA+degP,degB+degQ);

% if nargin ==4
if nargin ==3
    desiredPoles = -1*linspace(1,degC,degC);
end
[S c] = sylvester(Gplant,degP,degQ,desiredPoles);


%% fmincon
%inequality constraints
A = [];
b = [];
%equality constraints
Aeq = S;
beq = c;

sizeS = size(S);
IC = rand(sizeS(2),1);
objFun = @(x) norm(x);

lb = -inf*IC;
ub = inf*IC;
% nonlcon = @(x) roots(conv(denPlant, x(1:degQ+1)'));
%  nonlcon = @(x) [[roots(conv(denPlant, x(1:degQ+1)'))'*ones(length(roots(conv(denPlant, x(1:degQ+1)'))),1)];[0]];

nonlconFun = @(x) nonlcon(x,denPlant,degQ);
sol = fmincon(objFun,IC,A,b,Aeq,beq,lb,ub,nonlconFun);
% sol = fmincon(objFun,IC,A,b,Aeq,beq,lb,ub);
% sol = S\c;
%% analysis
numC = sol(1:degQ+1)'; %i.e. q(s)
denC = sol(degQ+2:degQ+2+degP)';
C = tf(numC,denC);
Gbar = Gplant+1/C;
% disp('Pole(Gbar)');
% disp(pole(Gbar));
% disp('Pole(1/Gbar)');
disp(pole(1/Gbar));
disp('nonlcon');
disp(nonlcon(sol,denPlant,degQ));