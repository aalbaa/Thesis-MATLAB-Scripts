% %open loop pllots
% 
% close all
% % bode(Gp1,Gp2,Gp3,Gp4);
% % legend('G_{p1}','G_{p2}','G_{p3}','G_{p4}');
% % title('$G_{p,i}$ plants bode plots','interpreter','latex');
% 
% figure('Name','Bode plots','pos',[250 75 900 600])
% bode(Gp1,Gp2,Gp3);
% legend('G_{p1}','G_{p2}','G_{p3}','interpreter','latex');
% title('$G_{p,i}$ plants bode plots','interpreter','latex');
% 
%%
figure('Name','Root locus plots','pos',[250 75 900 600])
subplot(2,4,1:2);
rlocus(Gp1);
title('$P_1$','interpreter','latex');

subplot(2,4,3:4);
rlocus(Gp2);
title('$P_2$','interpreter','latex');

subplot(2,4,6:7);
rlocus(Gp3);
title('$P_3$','interpreter','latex');
%%
% subplot(2,2,4);
% rlocus(Gp4);
% title('$G_{p4}$','interpreter','latex');
% 
% 
% % figure
% % subplot(2,2,1);
% % rlocus(Gbar1);
% % title('$\bar{G}_{2}$','interpreter','latex');
% % subplot(2,2,2);
% % rlocus(Gbar3);
% % title('$\bar{G}_{3}$','interpreter','latex');
% % subplot(2,2,3);
% % rlocus(Gbar4);
% % title('$\bar{G}_{4}$','interpreter','latex');
% %%
% figure('Name','Gbar1 plots','pos',[250 75 900 600])
% frequencyPlots(Gp1,Gff1);
% figure('Name','Gbar2 plots','pos',[250 75 900 600])
% frequencyPlots(Gp2,Gff2);
% figure('Name','Gbar3 plots','pos',[250 75 900 600])
% frequencyPlots(Gp3,Gff3);
% figure('Name','Gbar1sp1 plots','pos',[250 75 900 600])
% frequencyPlots(Gp1,Gff1sp1);
% figure('Name','Gbar1sp2 plots','pos',[250 75 900 600])
% frequencyPlots(Gp1,Gff1sp2);
% figure('Name','Gbar2sp1 plots','pos',[250 75 900 600])
% frequencyPlots(Gp2,Gff2sp1);
% figure('Name','Gbar2sp2 plots','pos',[250 75 900 600])
% frequencyPlots(Gp2,Gff2sp2);
% figure('Name','Gbar3sp2 plots','pos',[250 75 900 600])
% frequencyPlots(Gp3,Gff3sp2);