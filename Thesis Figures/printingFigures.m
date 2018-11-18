clear all;
close all;

G = tf([1 -2],[1 10 100]);

%figure
fg = figure;






% first bode plot
bodeMagThesis(G);
legItem{1} = ['$G(s)$'];


fg.PaperPositionMode = 'auto';

% %% frequency domain
% Gbar = polePlacement(G,1,2,1);  % CSP
% name = ['Figs/Bode_CSP'];

% [Gbar Cfb] = polePlacement(G,1,3,3); % optimization
% [Gbar2 Cfb2] = polePlacement(G,1,3,3,[-10,-20,-50,-80]); % optimization
% [Gbar3 Cfb3] = polePlacement(G,1,2,3); % optimization
% name = ['Figs/Bode_Comparison'];

% %% LMIs
% [Gbar Cfb] = lyapCL(G);
% name = ['Figs/Bode_SS'];
% 
% [Gbar Cfb] = lyapGenCL(G);
% name = ['Figs/Bode_DSS'];
% 
% lyapGenCL;
% name = ['Figs/Bode_DSS_improper'];


lyapGenCL;
[Gbar Gfb] = polePlacement(Psys,1,2,3,pole(1/Gbar));
name = ['Figs/Bode_DSS_improper_PP']; %Pole-Placement

%% Plots
% second bode plot
[h, xlbl, ylbl] = bodeMagThesis(Gbar);
legItem{2} = ['$\bar{G}(s)$'];
% [h2, xlbl2, ylbl2] = bodeMagThesis(Gbar2);
% legItem{3} = ['$\bar{G}(s)$ zeros: [-10 -20 -50 -80]. $G_{\mathrm{ff}}(s)$ order 1, rel. degree 2'];
% [h3, xlbl3, ylbl3] = bodeMagThesis(Gbar3);
% legItem{4} = ['$\bar{G}(s)$ zeros: [-1 -2 -3 -4]. $G_{\mathrm{ff}}(s)$ order 1, rel. degree 1'];



leg = legend(legItem);
leg.Interpreter = 'latex';
leg.FontSize = 12;
leg.Position = [0.15 0.78 0.1588 0.1174];

% print('Figs/Bode_CSP','-dpdf');
% print('Figs/Bode_CSP','-depsc');

print(name,'-dpdf');
print(name,'-depsc');


% 
% % NMP vs MP
% G = tf([1 1],[0.5 1]);
% Gbar = tf([1 -1],[0.5 1]);
% name = 'Figs/Bode_NMPvMP';
% 
% bode(G,'bo',Gbar,'rx');
% title('');
% 
xaxis = ['$\omega$ (rad/s)'];

xlab = xlabel(xaxis);

xlab.FontSize = 14;
xlab.Interpreter = 'latex';

% axis([0.1 100 -60 20])
% 
% 
% legItem{1} = 'Minimum Phase';
% legItem{2} = 'Nonminimum Phase';
% leg = legend(legItem);
% leg.Interpreter = 'latex';
% leg.FontSize = 14;
% leg.Position = [0.3 0.73 0.1588 0.1174];
% grid on;
% 
% print(name,'-dpdf');
% print(name,'-depsc');

% last1 = 57;
% last2 = 25;
% 
% [R, K] = rlocus(G);
% plt1 = plot(real(R(1,1:last1)),imag(R(1,1:last1)));
% plt1.LineWidth = 2;
% hold on;
% plt2 = plot(real(R(2,1:last2)),imag(R(2,1:last2)));
% plt2.LineWidth = 2;
% % grid on;
% 
% 
% 
% pls = pole(G);
% zer = pole(1/G);
% 
% pltPls1 = plot(real(pls(1)),imag(pls(1)),'bx');
% pltPls2 = plot(real(pls(2)),imag(pls(2)),'bx');
% pltzer = plot(real(zer),imag(zer),'bo');
% pltPls1.LineWidth = 2;
% pltPls1.MarkerSize = 13;
% pltPls2.LineWidth = 2;
% pltPls2.MarkerSize = 13;
% pltzer.LineWidth = 2;
% pltzer.MarkerSize = 13;
% 
% 
% xaxis = ['Re$\{s\}$ (Hz)'];
% xlab = xlabel(xaxis);
% xlab.FontSize = 14;
% xlab.Interpreter = 'latex';
% 
% yaxis = ['Im$\{s\}$ (Hz)'];
% ylab = ylabel(yaxis);
% ylab.FontSize = 14;
% ylab.Interpreter = 'latex';
% axis([-15,5,-10,10]);
% 
% t=-15:0.01:10;
% plot(zeros(length(t),1),t,'k--');
% plot(t,zeros(length(t),1),'k-.');
%  name = ['Figs/Bode_rlocus'];
%  print(name,'-dpdf');
% print(name,'-depsc');