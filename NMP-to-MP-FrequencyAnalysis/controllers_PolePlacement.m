%%%%%%% controller using pole placement%%%%%%%
%Used on plants 1 and 2 in TFs
% Ap2*[q1 q0 p1 p0]' = d2;
%%
%plant 2
Ap1 = [0 0 1 0;
    1 0 a 1;
    -b 1 0 a;
    0 -b 0 0];
%%
Ap2 = [0 0 1 0;
    1 0 -a 1;
    -b 1 0 -a;
    0 -b 0 0];

%%
%coefficent of the polynomical where we want to place our zeros
%i.e. expansion of (s+root1)(s+root2)(s+root3)
% (s+1)(s+2)(s+3) expands to :
d2 = [1 6 11 6]';

P2 = Ap2\d2;

P1 = Ap1\d2;

G01 = tf(P1(1:2)',P1(3:4)');

G02 = tf(P2(1:2)',P2(3:4)');
%%
%Now to find the max gain such that all roots of Gpi*Cpi are on the OLHP
k1 = maxGain(Gp1*G01);
k2 = maxGain(Gp2*G02);

Gff1 = 1/(k1*G01);
Gff2 = 1/(k2*G02);

Gbar1 = Gp1+Gff1;
Gbar2 = Gp2+Gff2;

