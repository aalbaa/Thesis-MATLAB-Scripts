% This script is simply to demonstrate how to use YALMIP, mosek
%use :  opt = sdpsettings('solver','mosek','verbose',0);solvesdp(F,gamma,opt);
%       to use 'mosek'

% %% Lyapunov Function
% % in a dynamical system x_dot = Ax+Bu. A is stable iff A'*P+P*A<0 for some
% % P>0
% 
% A = [1 2;-2 -5];        %unstable
% n = size(A,1);
% 
% %setting the variables (P in this case) 
% %sdp stands fro (semi-definite programming)
% %other options: sdpvar(n,n,'full','real');
% %sdpvar(n,n,'hermitian','complex')
% P = sdpvar(n);      %sdpvar(n) <=> sdpvar(n,n,'symmetric','real');
% 
% eps = 1e-4; %since exact inequalities are not realizable. 
%     %(used when setting the inequality)
% 
% epsEye = eps*eye(size(A,1));
% 
% %Lyapunov eqn
% LMI1 = A'*P+P*A <= -epsEye;
% LMI2 = P >= epsEye;
% F = [LMI1, LMI2];
% 
% %solving the contraint problem;
% solvesdp(F);
% 
% Pval = double(P);
% Q = double(LMI1);
% if (double(LMI1) && double(LMI2))
%     disp('A is stable');
% else
%     disp('A is unstable');
% end


% 
% %% H_inf norm:
% %given  xDot = Ax + Bu;
% %       y    = Cx + Du;
% %The H_infty norm solved by solving a constrained minimzation problem
% %Check literature for the LMI. It's used here
% clear all;
% 
% % g = tf(1,[1 2 1]);
% g = [0 tf([3 0],[1 1 10]);tf([1 1],[1 5]),tf(2,[1 6])];
% 
% G = ss(g);
% A = G.A;
% B = G.B;
% C = G.C;
% D = G.D;
% 
% n = size(A,1);
% 
% gamma = sdpvar(1); %the function to be minimized;
% P = sdpvar(n);
% 
% eps = 1e-4;
% 
% 
% LMI11 = A'*P + P*A;
% LMI12 = P*B;
% LMI13 = C';
% LMI21 = LMI12';
% LMI22 = -gamma*eye(size(D',1));
% LMI23 = D';
% LMI31 = LMI13';
% LMI32 = LMI23';
% LMI33 = LMI22;
% 
% LMI = [ LMI11, LMI12, LMI13;
%         LMI21, LMI22, LMI23;
%         LMI31, LMI32, LMI33];
% F =  LMI <= -eps*eye(size(LMI,1));
% solvesdp(F,gamma);
% 
% if (double(F))
%     printf("H_infty : %0.10f\n",double(gamma));
% else
%     disp('no solution');
% end

%% State feedback control
%Given an unstable plant, find K s.t A+BK is stable
% (A+BK)'*P+P*(A+BK)<0 <=> A'*P+P*A+K'*B'*P+P*BK
% Let Q = inv(P) and multily and premultiply the inequality by Q
% we get <=> QA'+AQ+QK'B'+BKQ
% set L:= KQ then 
%           QA'+AQ+L'B+BL < 0 is an LMI
clear all;

g = tf(1,[1 -1 1]); %clearly unstable
G = ss(g);
A = G.A;
B = G.B;
C = G.C;
D = G.D;

n = size(A,1);

Q = sdpvar(n); %same size as P
L = sdpvar(1,n,'full','real');

eps = 1e-4;
epsEye = eps*eye(n);

LMI1 = Q*A'+A*Q+L'*B'+B*L <= -epsEye;
LMI2 = Q >= epsEye;

F = [LMI1, LMI2];

solvesdp(F);

if (double(LMI1) && double(LMI2))
    disp('K is :');
    K = double(L)*inv(double(Q))
    disp('eigenvalues of CL: ');
    eig(A+B*K)
else
    disp('no solution');
end

