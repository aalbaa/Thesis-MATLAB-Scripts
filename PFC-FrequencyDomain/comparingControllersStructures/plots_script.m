close all;
%% Font sizes
% Font size, line size, and line width. 
font_size = 15;
line_size = 15;
line_width = 2;
position = [250 75 900 600];

% 
% 
% %% Plant 1
% figure('Name','BodeGp1biproper','pos',position);
% %static gain not possible
% bode(Gp1,Gbar111,Gbar122,Gbar122_2,Gbar122_3);
% leg1=legend('$G_{p1}$','$\tilde{G}$ bi-proper deg 1','$\tilde{G}$ bi-proper deg 2 poles at [-1 -2 -3]','$\tilde{G}$ bi-proper deg 2 poles at [-100 -200 -300]','$\tilde{G}$ bi-proper deg 2 poles at [-0.1 -0.2 -0.3]');%$,'Interpreter','latex');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p1}$ plots for bi-proper controllers','Interpreter','latex','fontsize',font_size);



% 
% figure('Name','BodeGp1strictlyProper','pos',position);
% %static gain not possible
% bode(Gp1,Gbar101,Gbar102,Gbar113);
% leg1=legend('$G_{p1}$','$\tilde{G}$ contr. den.deg = 0 num.deg = 1 ','$\tilde{G}$ contr. den.deg = 0 num.deg = 2 ','$\tilde{G}$ contr. den.deg = 1 num.deg = 3 ');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p1}$ plots for strictly proper controllers','Interpreter','latex','fontsize',font_size);
% 

figure('Name','BodeGp1proper','pos',position);
%static gain not possible
bode(Gp1,Gbar122,Gbar102);
leg1=legend('$P(s)$','$\tilde{P}(s)$ using bi-proper $G_{ff}(s)$','$\tilde{P}(s)$ using strictly proper $G_{ff}(s)$');
set(leg1,'Interpreter','latex');
set(leg1,'FontSize',11);
title('Comparison of different $\tilde{P}(s)$','Interpreter','latex','fontsize',font_size);
% 
% 
% %% Plant 2
% figure('Name','BodeGp1biproper','pos',position);
% %static gain not possible
% bode(Gp2,Gbar211,Gbar222);%,Gbar222_2,Gbar222_3);
% leg1=legend('$G_{p2}$','$\tilde{G}$ bi-proper deg 1','$\tilde{G}$ bi-proper deg 2 poles at [-1 -2 -3]','$\tilde{G}$ bi-proper deg 2 poles at [-100 -200 -300]','$\tilde{G}$ bi-proper deg 2 poles at [-0.1 -0.2 -0.3]');%$,'Interpreter','latex');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p2}$ plots for bi-proper controllers','Interpreter','latex','fontsize',font_size);
% 
% figure('Name','BodeGp2strictlyProper','pos',position);
% %static gain not possible
% bode(Gp2,Gbar201,Gbar202,Gbar213);
% leg1=legend('$G_{p2}$','$\tilde{G}$ contr. den.deg = 0 num.deg = 1 ','$\tilde{G}$ contr. den.deg = 0 num.deg = 2 ','$\tilde{G}$ contr. den.deg = 2 num.deg = 4 ');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p2}$ plots for strictly proper controllers','Interpreter','latex','fontsize',font_size);
% 
% 
% figure('Name','BodeGp2proper','pos',position);
% %static gain not possible
% bode(Gp2,Gbar222,Gbar202);
% leg1=legend('$G_{p2}$','$\tilde{G}$ bi-proper deg 2 poles at [-1 -2 -3]','$\tilde{G}$ contr. den.deg = 0 num.deg = 2 ');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p2}$ plots for bi-proper  and strictly proper controllers','Interpreter','latex','fontsize',font_size);
% 
% % 
% 
% % Plant 3
% figure('Name','BodeGp1biproper','pos',position);
% bode(Gp3,Gbar300,Gbar311,Gbar322);%,Gbar322_2,Gbar322_3);
% leg1=legend('$G_{p3}$','$\tilde{G}$ static gain','$\tilde{G}$ bi-proper deg 1','$\tilde{G}$ bi-proper deg 2 poles at [-1 -2 -3]','$\tilde{G}$ bi-proper deg 2 poles at [-100 -200 -300]','$\tilde{G}$ bi-proper deg 2 poles at [-0.1 -0.2 -0.3]');%$,'Interpreter','latex');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p3}$ plots for bi-proper controllers','Interpreter','latex','fontsize',font_size);
% 
% figure('Name','BodeGp3strictlyProper','pos',position);
% %static gain not possible
% bode(Gp3,Gbar301,Gbar302,Gbar313);
% leg1=legend('$G_{p3}$','$\tilde{G}$ contr. den.deg = 0 num.deg = 1 ','$\tilde{G}$ contr. den.deg = 0 num.deg = 2 ','$\tilde{G}$ contr. den.deg = 1 num.deg = 3 ');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p3}$ plots for strictly proper controllers','Interpreter','latex','fontsize',font_size);
% 
% 
% figure('Name','BodeGp3proper','pos',position);
% %static gain not possible
% bode(Gp3,Gbar322,Gbar301,Gbar302);
% leg1=legend('$G_{p3}$','$\tilde{G}$ bi-proper deg 2 poles at [-1 -2 -3]','$\tilde{G}$ contr. den.deg = 0 num.deg = 1 ','$\tilde{G}$ contr. den.deg = 0 num.deg = 2 ');
% set(leg1,'Interpreter','latex');
% set(leg1,'FontSize',11);
% title('Comparison of different $\tilde{G}_{p3}$ plots for bi-proper  and strictly proper controllers','Interpreter','latex','fontsize',font_size);
% 


% 
% figure('Name','freqPlotsGbar111','pos',position);
% frequencyPlots(Gp1,Gff111);
% 
% figure('Name','freqPlotsGbar122','pos',position);
% frequencyPlots(Gp1,Gff122);
% 
% figure('Name','freqPlotsGbar122_2','pos',position);
% frequencyPlots(Gp1,Gff122_2);
% 
% figure('Name','freqPlotsGbar122_3','pos',position);
% frequencyPlots(Gp1,Gff122_3);
% 
% figure('Name','freqPlotsGbar101','pos',position);
% frequencyPlots(Gp1,Gff101);
% 
% figure('Name','freqPlotsGbar102','pos',position);
% frequencyPlots(Gp1,Gff102);
% 
% figure('Name','freqPlotsGbar113','pos',position);
% frequencyPlots(Gp1,Gff113);
% 
%

% 
% figure('Name','freqPlotsGbar211','pos',position);
% frequencyPlots(Gp2,Gff211);
% 
% figure('Name','freqPlotsGbar222','pos',position);
% frequencyPlots(Gp2,Gff222);
% 
% figure('Name','freqPlotsGbar201','pos',position);
% frequencyPlots(Gp2,Gff201);
% 
% figure('Name','freqPlotsGbar202','pos',position);
% frequencyPlots(Gp2,Gff202);
% 
% figure('Name','freqPlotsGbar224','pos',position);
% frequencyPlots(Gp2,Gff213);
% 

% 
% figure('Name','freqPlotsGbar300','pos',position);
% frequencyPlots(Gp3,Gff300);
% 
% figure('Name','freqPlotsGbar311','pos',position);
% frequencyPlots(Gp3,Gff311);
% 
% figure('Name','freqPlotsGbar322','pos',position);
% frequencyPlots(Gp3,Gff322);
% 
% figure('Name','freqPlotsGbar301','pos',position);
% frequencyPlots(Gp3,Gff301);
% 
% figure('Name','freqPlotsGbar302','pos',position);
% frequencyPlots(Gp3,Gff302);
% 
% figure('Name','freqPlotsGbar313','pos',position);
% frequencyPlots(Gp3,Gff313);
% 

