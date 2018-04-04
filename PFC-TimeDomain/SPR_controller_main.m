% this script is to get a SPR controller that renders the closed loop of
% the plant AS

% clear all;
close all;

Ptf = tf([1 -2],[1 10 100]);
[Ap, Bp, Cp, Dp] = ssdata(Ptf);

np = size(Ap,1);
nc = np;

% initial params
Cc = ones(1,nc); % assuming some arbitrary structure
Dc = 1;


%%% solver params
solver = 1;
verb = 0;
eps = 1e-5;

bode(Ptf);

legInfo{1} = ['$P(s)$'];


hold on;

iterations = 3;
for i=1:iterations
    lyapCL;
    KYP_LMI;
    disp('Cc; ');
    disp(Cc);
    disp('Dc: ');
    disp(Dc);  
    
    if i >=45
        bode(Gbar);    
        legInfo{i+1} = ['$\bar{G}(s) $ ' num2str(i)];
    end
end
leg = legend(legInfo);
leg.Interpreter = 'latex';

hold off;
