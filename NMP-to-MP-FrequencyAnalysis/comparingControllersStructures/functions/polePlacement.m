%Amro Al-Baali
%November 25, 2017
%amro.al-baali@mail.mcgill.ca
% function S = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
% function [C Gfb Gbar] = polePlacement(denPlant,numPlant,degP,degQ,desiredPoles)
function [C Gfb Gbar] = polePlacement(Gplant,degP,degQ,desiredPoles)

   
[S c]=sylvester(Gplant,degP,degQ,desiredPoles);
% C=[Tnum Tden];
% if size(S)==size(S') 
%     sol = S\c;
% else
%     sol = pinv(S)*c;
%     sol = lsqminnorm(S,c,eps);
    sol = S\c;
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