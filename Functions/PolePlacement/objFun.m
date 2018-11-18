function objFunc = objFun(x,degP,degQ)

Gff = tf(x(degQ+2:degQ+2+degP)',x(1:degQ+1)');

% Tl=tf([1],[0.1 1]);
% % Th=tf([1 0],[1e-5 0.1 1]);
% Th=tf([1 0],[1e-2 1e3 1e-3]);
% % Th=tf([1 0],[1e-10 1e5 1]);

% W = Th;
% objFunc = norm(W*Gff);
objFunc = norm(Gff);

