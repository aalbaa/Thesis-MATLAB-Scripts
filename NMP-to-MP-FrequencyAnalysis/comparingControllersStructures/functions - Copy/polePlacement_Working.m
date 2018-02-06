%Amro Al-Baali
%November 25, 2017
%amro.al-baali@mail.mcgill.ca
% function S = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
% function [C Gfb Gbar] = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
function [Gbar C Gfb] = polePlacement(Gplant,degP,degQ,desiredPoles)
   
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
   
[S c]=sylvester(Gplant,degP,degQ,desiredPoles);
% C=[Tnum Tden];
% if size(S)==size(S') 
%     sol = S\c;
% else
%     sol = pinv(S)*c;
%     sol = lsqminnorm(S,c,eps);
    sizeS = size(S);
    IC = rand(sizeS(2),1);
    A =[];
    b = [];
    sol = fmincon(@(x) 1,IC,A,b,S,c);
%     sol = S\c;
% end
%% sol
Gp = Gplant;%tf(numPlant,denPlant);
C = tf(sol(1:degQ+1)',sol(degQ+2:degQ+2+degP)');
Gfb = feedback(Gp,C);
Gbar = Gp+1/C;
% [C Gfb Gbar] = [Gc Gfb Gp+1/Gc];
% C = Gc;
% disp(pole(Gfb));
% disp(S*sol);
end