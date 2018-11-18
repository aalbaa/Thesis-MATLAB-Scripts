clear all;
close all;

TFs;

Gp = Gp4;
% Gp = tf(poly([1 2]),poly([-6 -7 -9]));
% [Gbar1 C1 Gfb sol] = polePlacement(Gp,1,2,1);
% [Gbar2 C2 Gfb sol] = polePlacement(Gp,1,2,2);
[Gbar3 C3 Gfb sol IC S c] = polePlacement(Gp,1,2,3);

% RH = rhStabilityCriterion(conv(sol(1:2),Gp3.denominator{1,1}));

% C2 = maxGain(C1*Gp)*C1;
% Gbar2 = Gp+1/C2;
% 
% disp('pole(Gbar1):');
% disp(pole(Gbar1));
% disp('pole(1/Gbar1):');
% disp(pole(1/Gbar1));
% disp('pole(Gbar2):');
% disp(pole(Gbar2));
% disp('pole(1/Gbar2):');
% disp(pole(1/Gbar2));
disp('pole(Gbar3):');
disp(pole(Gbar3));
disp('pole(1/Gbar3):');
disp(pole(1/Gbar3));


disp('IC: ');
disp(IC);
% bode(Gbar1,Gbar2,Gbar3,Gp);
% leg1 = legend('$\tilde{G}_1$  min 1','$\tilde{G}_2 min ||x||$  ','$\tilde{G}_3 min ||G_{ff}||$','$G_p $');
B = bodemag(Gbar3,Gp);
title();
legItem{1} = ['$\bar{G}(s)$'];
legItem{2} = ['$G(s)$'];
leg1 = legend(legItem);
leg1.FontSize = 12;
leg1.Interpreter = 'latex';

% set(leg1,'interpreter','latex');