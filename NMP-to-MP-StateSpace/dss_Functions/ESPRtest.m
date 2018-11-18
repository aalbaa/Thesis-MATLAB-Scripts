% 1 if true
% 0 if false
function [result] = ESPRtest(Ptf)

if any(pole(Ptf)<0)
    result = 0;
    return;
else
    