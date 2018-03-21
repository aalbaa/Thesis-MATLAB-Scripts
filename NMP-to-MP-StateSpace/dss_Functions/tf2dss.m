% Amro Al Baali
% March 17, 2018
%
% This function takes the numerator and denominator of a transfer function
% and converts them to Descriptor State Space

% March 20 16:00:   Changed the input from (Num,Den) to (SYS) (the TF)
%                   Changed output from [A,B,C,D] to [SYS_DSS,A,B,C,D]

function [SYS_DSS, A,B,C,D,E] = tf2dss(SYS)
Num = SYS.Numerator{1,1};
Den = SYS.Denominator{1,1};

Num=Num(find(Num>0,1):end); % remove leading zeros
Den=Den(find(Den>0,1):end); % remove leading zeros

% if all(Den == 0)
%     disp("denomenator can't be zero");
%     return;
% end
%not necessary anymore since an error will pop when creating a TF: SYS that
%has zero as its den.

p = length(Num)-1;
n = length(Den)-1;

DenF = flip(Den);

if Den(1) ~= 0
    Ass = [[zeros(n-1,1), eye(n-1)];
            -DenF(1:max(n,1))./Den(1)];
    Bss = [zeros(n-1);1./Den(1)];
else
    Ass = [[zeros(n-1,1), eye(n-1)];
        -DenF(1:max(n,1))];
    Bss = [zeros(n-1);1];
end


n2 = size(Ass,1);

A = [[Ass, zeros(n2,p-n2+1)]; zeros(p-n2+1,n2),eye(p-n2+1)];
B = [Bss;zeros(p-n2+1,1)];
E = [eye(n) zeros(n2,p-n+1); zeros(p-n2+1,n2-1) eye(p-n2+1) zeros(p-n2+1,1)];
C = flip(Num);
D = 0;

SYS_DSS = dss(A,B,C,D,E);
end
