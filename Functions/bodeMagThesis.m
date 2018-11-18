% this function plots the magnitude of the Bode plot professionally
% close all;

% G = tf([1 -2],[1 10 100]);
function [handle, xlbl, ylbl] = bodeMagThesis(G);

dx = 0.01;
omegaFirst = 1e-1;
omegaLast = 1e2;
win = omegaFirst:dx:omegaLast;

[mag, ~, wout] = bode(G,win);


mag = squeeze(mag); % this is NOT in dB
mag = 20*log10(mag); % in dB

wout = squeeze(wout);

% fg = figure;
plt = semilogx(wout,mag);
grid on;

plt.LineWidth = 2.5;

xaxis = ['$\omega$ (rad/s)'];
yaxis = ['Magnitude (dB)'];

xlab = xlabel(xaxis);
ylab = ylabel(yaxis);

xlab.FontSize = 18;
xlab.Interpreter = 'latex';

ylab.FontSize = 14;
ylab.Interpreter = 'latex';
ylab.FontWeight = 'bold';


hold on;

handle = plt;
xlbl = xlab;
ylbl = ylab;
end