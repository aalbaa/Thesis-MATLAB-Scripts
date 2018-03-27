%Amro Al-Baali
%November 25, 2017
%amro.al-baali@mail.mcgill.ca
% Input:
%       Gp: Plant
%       degP: degree of the denominator of the (feedback) controller
%       degQ: degree of the numerator of the (feedback) controller

% function S = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
% function [C Gfb Gbar] = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
%optn: 1== min norm(Gff), 2== min norm(Gp-Gff)
function [Gbar C Gfb sol IC] = polePlacement(Gplant,degP,degQ,optn,desiredPoles)
   
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
    desiredPoles = -1*linspace(1,degC,degC); %if no desired poles were entered
    optn = 1;
elseif nargin ==4
    desiredPoles = -1*linspace(1,degC,degC);
end


%pole placement (for the denominator of feedback loop) 
% which happens to be the numerator of Gbar Sx=c
[S c]=sylvester(Gplant,degP,degQ,desiredPoles);

% C=[Tnum Tden];
% if size(S)==size(S') 
%     sol = S\c;
% else
%     sol = pinv(S)*c;
%     sol = lsqminnorm(S,c,eps);
    sizeS = size(S);
    IC = rand(sizeS(2),1);
%     IC = S\c;
    A =[];
    b = [];
%     sol = fmincon(@(x) 1,IC,A,b,S,c);

lb = -inf*IC;
ub = inf*IC;
% nonlconFun = @(x) nonlcon(x,denPlant,degQ, S);
nonlconFun = @(x) nonlcon2(x,Gplant,degP,degQ);


options = optimoptions('fmincon');
% options.Algorithm ='sqp';
options.MaxFunctionEvaluations = 1e4;
options.MaxIterations = 1e4;
options.ConstraintTolerance = 1e-10;
% sol = fmincon(@(x) 1,IC,[],[],S,c);

% Gffx = @(x)  tf(x(1:degQ+1)',x(degQ+2:degQ+2+degP)');

if optn == 1
    objFunc = @(x) 1;
elseif optn ==2 
    objFunc = @(x) norm(x);
else
    objFunc = @(x) objFun(x,degP,degQ);
end




sol = fmincon(@(x) objFunc(x),IC,[],[],S,c,lb,ub,nonlconFun,options);

%     sol = S\c;
% end
%% sol
Gp = Gplant;%tf(numPlant,denPlant);
C = tf(sol(1:degQ+1)',sol(degQ+2:degQ+2+degP)');
% C = maxGain(C*Gp)*C;
Gfb = feedback(Gp,C);
Gbar = Gp+1/C;

end