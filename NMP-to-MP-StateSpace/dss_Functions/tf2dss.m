% Amro Al Baali
% March 17, 2018
%
% This function takes the numerator and denominator of a transfer function
% and converts them to Descriptor State Space

function [A,B,C,D,E] = tf2dss(Num,Den)

if Den == 0
    disp("denomenator can't be zero");
    return;
end

p = length(Num)-1;
n = length(Den)-1;

DenF = flip(Den);

Ass = [[zeros(n-1,1), eye(n-1)];
        -DenF(1:max(n,1))./Den(1)];
Bss = [zeros(n-1);1./Den(1)];

n2 = size(Ass,1);

A = [[Ass, zeros(n2,p-n2+1)]; zeros(p-n2+1,n2),eye(p-n2+1)];
B = [Bss;zeros(p-n2+1,1)];
E = [eye(n) zeros(n2,p-n+1); zeros(p-n2+1,n2-1) eye(p-n2+1) zeros(p-n2+1,1)];
C = flip(Num);
D = 0;
