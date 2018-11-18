% Gp is the plant,
% Gff is the parallel feedforward controller
% G0 is the feedback controller

function [x] = frequencyPlots(Gp,Gff)
% Gff=1/Gff;
G0=1/Gff;
Gbar=Gp+Gff;
L=Gp*G0;
if isproper(L)==0
    L=1/L;
end

subplot(2,2,1);
nyquist(L);

subplot(2,2,2);
nyqlog(L);

subplot(2,2,3);
bode(Gp,Gbar);
legend('Gp','Gbar');

subplot(2,2,4);
rlocus(L)

x = [Gbar Gff];
