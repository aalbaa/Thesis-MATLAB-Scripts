function [c,ceq] = nonlin(x,den,degQ)
c = real(roots(conv(den,x(1:1:degQ+1))));     % Compute nonlinear inequalities at x.
ceq = [];   % Compute nonlinear equalities at x.

% function [c,ceq] = nonlin(x)
% c = (x(1)-1/3)^2 + (x(2)-1/3)^2 - (1/3)^2;
% ceq = [];