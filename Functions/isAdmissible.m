% This script is to test the admissibility of (E,A)

Ptf = tf([1 -1],[1 2 1]); % this is admissible

[A, B, C, D, E] = dssdata(Ptf);

n = size(A,1);

P = sdpvar(n);



eps = 1e-5;
% M = A'*P*A - E'*P*E;
% 
% if ~any(any(isnan(double(M)))) && any(eig(double(M)) >=0)
%     disp('infeasible');
% end
%     
% LMI1 = [M  <= -eps];
% 
% % gamma = sdpvar(1);
% % 
% % H11 = A'*P*A-E'*P*E+C'*C;
% % H12 = A'*P*B+C'*D;
% % H21 = B'*P*A + D'*C;
% % H22 = -gamma+B'*P*B+D'*D;
% % 
% % LMI1 = [[H11, H12;
% %         H21, H22] <= -eps];
% % LMI2 = [E'*P*E >= 0];
% 
% % LMI1 = [A'*P*A <= -eps+ E'*P*E];
% LMI2 = [E'*P*E >= 0];
% 
% gamma = [];
% 
% F = [LMI1, LMI2];

% 
% P = sdpvar(n);
% gamma = sdpvar(1);
% 
% H11 = A'*P*A-E'*P*E+C'*C;
% H12 = A'*P*B+C'*D;
% H22 = B'*P*B+D'*D;
% 
% LMI1 = [[H11, H12;
%         H12', H22] <= -eps];
% LMI2 = [E'*P*E >= 0];
% 
% LMI3 = [gamma >= eps];
% 
% t=[];
% F = [LMI2, LMI1];


X = sdpvar(n);
Y = sdpvar(n);

eps = 1e-5;

LMI1 =[E'*X*A+A'*X*E == -E'*Y*E];
% LMI1 =[E'*X*E+A'*X*E <= -eps];
LMI2 = [Y >= eps];
LMI3 = [X >= 0];

F = [LMI1, LMI2, LMI3];
t = [];

%% Solving the sdp



solver = 1;
verb = 1;

opt = sdpsettings;
opt.verbose = verb;
% opt.removeequalities = 1;

if solver == 1    
    opt.solver = 'mosek';
    opt.mosek.MSK_DPAR_ANA_SOL_INFEAS_TOL = 1e-3;
elseif solver == 2       
    opt.solver = 'sdpt3';
elseif solver == 3        
    opt.solver = 'sedumi';
else
    opt = [];
end


sol=solvesdp(F,t,opt);

