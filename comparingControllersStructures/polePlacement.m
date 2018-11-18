%Amro Al-Baali
%November 25, 2017
%amro.al-baali@mail.mcgill.ca
% function S = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
% function [C Gfb Gbar] = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
function [C Gfb Gbar] = polePlacement(Gplant,degP,degQ,desiredPoles)
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
%% Given
% b=[1 -1]; %numerator
% a=[1 -2 0]; %denomenatr
% degP=1; %controller of controller's denomenatr
% degQ=1; %degree of controller's numerator
% c = [1 6 11 6]'; %desired poles
% c=desiredPoles;
c = poly(desiredPoles)';
%% operations

%degA>=degB
% if length(numPlant)<length(denPlant)
%     numPlant=[zeros(1,degA-degB) numPlant];
% elseif length(numPlant)>length(denPlant)
%     denPlant=[zeros(1,degB-degA) denPlant];
% end

Tden = toeplitz([denPlant zeros(1,degP)],[denPlant(1) zeros(1,degP)]);
Tnum = toeplitz([numPlant zeros(1,degQ)],[numPlant(1) zeros(1,degQ)]);
% if degQ<degP
%     Tnum= [zeros((degP-degQ),degQ+1);Tnum];
% elseif degQ>degP
%     Tden= [zeros((degQ-degP),degP+1);Tden];
% end

if (degQ+degB) > (degP+degA)
    Tden = [zeros(degQ+degB-degP-degA,degP+1);Tden];
elseif (degQ+degB) < (degP+degA)
    Tnum = [zeros(-degQ-degB+degP+degA,degQ+1);Tnum];
end
        

    

S=[Tnum Tden];


% C=[Tnum Tden];
% if size(S)==size(S') 
%     sol = S\c;
% else
%     sol = pinv(S)*c;
%     sol = lsqminnorm(S,c,eps);
    sol = S\c;
% end
%% sol
Gp =tf(numPlant,denPlant);
C = tf(sol(1:degQ+1)',sol(degQ+2:degQ+2+degP)');
Gfb = feedback(Gp,C);
Gbar = Gp+1/C;
% [C Gfb Gbar] = [Gc Gfb Gp+1/Gc];
% C = Gc;
% disp(pole(Gfb));
% disp(S*sol);
end