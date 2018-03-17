% Amro Al Baali
% February 13, 2018
%
%
% Example Of using h2syn(P,sizeB2,sizeC2)
% Optimal Control formulation:
%       x_dot   =   Ax  +   B1w  +  B2u;
%       z       =   C1x +   D11w +  D12u;
%       y       =   C2x +   D21w +  D22u;
%Where  x are the states,
%       w is the exegenous input
%       u is the plant's input from controller (output of controller) 
%       z is the exegenous output
%       y is the output of plant (input to controller)
%
% A REGULATOR is used in this example (from Prof. J. R. Forbes' slides of
% MECH 513)
% Where     A   =   Aplant;     B1  =   0;      B2  =   Bplant;
%           C1  =   [-C; 0];    D11 =   [1;0];  D12 =   [-Dplant;1];
%           C2  =   -C;         D21 =   1;      D22 =   [-Dplant];
% With 0 and 1 being matrices of appropriate dimensions
%
% In this script, we demonstrate the result in time domain and in frequency
% domain
%

clear all;
close all;
home;

%number of exegenous inputs
wSize = 1;

%Transfer function used

PlantNumerator = [1 -2];
PlantDenominator = [1 10 100];
Gtf = tf(PlantNumerator,PlantDenominator);

% state space
[Ag Bg Cg Dg] = tf2ss(PlantNumerator,PlantDenominator);

% Constructing the regulator-formulation matrices
% 
% A = Ag;
% 
% B1 = zeros(size(A,1),wSize);   
% % B1 = Bg;
% B2 = Bg;
% 
% C1 = [-Cg;zeros(size(Cg,1),size(A,2))];
% C2 = -Cg;
% 
% D11 = [-eye(size(Cg,1),wSize); zeros(size(Cg,1),wSize)];
% D12 = [-Dg;eye(size(Cg,1),size(Bg,2))];
% D21 = eye(size(Cg,1),wSize);
% D22 = -Dg;


A = Ag;

% B1 = zeros(size(A,1),wSize);  
B1 = Bg;
B2 = Bg;

C1 = [-Cg];%;zeros(size(Cg,1),size(A,2))];
C2 = -Cg;

D11 = 0;%[ones(size(Cg,1),wSize); zeros(size(Cg,1),wSize)];
D12 = [-Dg];%;zeros(size(Cg,1),size(Bg,2))];
D21 = zeros(size(Cg,1),wSize);
% D21 = [0 0];
D21 = [zeros(size(Cg,1),wSize-size(Dg,2)), Dg];
D22 = 0;
% 





% 
% 
% %constructing P (for optimal control formulation)
% 
% P = ss(Ag,[B1 B2], [C1;C2], [D11 D12; D21 D22]);
% 
% %finding the optimal controller K, and the closed loop ss CL
% 
% [K1 CL] = h2syn(P,size(B2,2),size(Cg,1));
% 
% 
% %controller transfer function
% [Cnum, Cden] = ss2tf(K1.A,K1.B,K1.C,K1.D);
% Ctf1 = tf(Cnum,Cden);
% 
% 
% %closed loop
% Gcl = feedback(Gtf,Ctf1);
% 
% disp("Poles of Gcl: ");
% disp(pole(Gcl));
% 
% disp("Poles of CL (ss): ");
% disp(eig(CL.A));
%         

A = Ag;
B = Bg;
C = Cg;
D = Dg;

% X1 = sdpvar(size(A,1),size(A,1),'full');
% X2 = sdpvar(size(A,1),size(A,1),'full');
% Y1 = sdpvar(size(C1,2),size(C1,2),'full');
% Y2 = sdpvar(size(C1,2),size(C1,2),'full');
% Z = sdpvar(size(C1,1),size(C1,1),'full');
% gamma = sdpvar(1,1);

X1 = sdpvar(size(A,1),size(A,1));
% X2 = sdpvar(size(A,1),size(A,1));
Y1 = sdpvar(size(C1,2),size(C1,2));
% Y2 = sdpvar(size(C1,2),size(C1,2));
Z = sdpvar(size(C1,1),size(C1,1));
gamma = sdpvar(1,1);



numStates = 2;
numInputs = 1;
numOutputs = 1;
An = sdpvar(numStates,numStates);
Bn = sdpvar(numStates,numInputs);
Cn = sdpvar(numOutputs,numStates);
Dn = sdpvar(numOutputs,numInputs);
% Dn = 0;


%%% LMI 1;
E11 = A*Y1+Y1*A'+B2*Cn + Cn'*B2';
E12 = A+An'+B2*Dn*C2;
E13 = B1+B2*Dn*D21;
E22 = X1*A+A'*X1+Bn*C2+C2'*Bn';
E23 = X1*B1+Bn+Bn*D21;

LMI1 = [E11 E12 E13;
        E12' E22 E23;
        E13' E23' -1];
    
<<<<<<< HEAD
F = [LMI1<0];
solvesdp(F,gamma);
=======
F11 = Y1;
F12 = eye(size(Y1,1),size(X1,2));
F13 = Y1'*C1'+Cn'*D12';
F22 = X1;
F23 = C1'+C2'*Dn*D12';
F33 = Z;

LMI2 = [F11 F12 F13;
        F12' F22 F23;
        F13' F23' F33];

LMI3 = D11+D12*Dn*D21;
LMI4 = [X1 eye(size(X1,1),size(Y1,2));
    eye(size(X1,1),size(Y1,2)) Y1];
LMI5 = trace(Z);
% LMI6 = X2*Y2+X1*Y1-eye(size(X1,1),size(Y1,2));


        
    
F = [LMI1<0, LMI2>0,LMI4>0, LMI5<gamma, LMI3==0];%, LMI6==0];
% F = [LMI1 == 0];
opt = sdpsettings('solver','mosek','verbose',0);
solvesdp(F,gamma,opt);
% solvesdp(F,[],opt); 

X1 = double(X1);
Y1 = double(Y1);

% X2Y2 = sdpvar(size(X1,1),size(Y1,2));
% LMI6 = [X2Y2 + X1*Y1-eye(size(X1,1),size(Y1,2)) == 0];
% solvesdp(LMI6,[],opt);

% X2 = fliplr(double(X2Y2));
% Y2 = fliplr(eye(2));

X2 = -(X1*Y1-eye(size(X1,1),size(Y1,2)));
Y2 = eye(2);

M1 = inv([X2 X1*B2;zeros(1,2) eye(1,1)]);
M2 = ([An Bn;Cn Dn]- [X1*A*Y1 zeros(size(X1,1),size(Bn,2));zeros(size(Cn,1),size(Y1,2)),zeros(size(Cn,1),size(Bn,2))]);
M3 = inv([Y2' zeros(size(Y2,2),size(Bn,2)); C2*Y1 eye(size(C2,1),size(Bn,2))]);


K = double(M1*M2*M3);
Ak = K(1:2,1:2);
Bk = K(1:2,3);
Ck = K(3,1:2);
Dk = K(3,3);


Dc = inv(1+Dk*D22)*Dk;
Cc = (eye(size(Dc,1)) - Dc*D22)*Ck;
Bc = Bk*(eye(size(Dc,1))-Dc*D22);
Ac = Ak-Bc*inv(eye(size(D22,1))-D22*Dc)*D22*Cc;


Css = ss(Ac,Bc,Cc,Dc);

Ctf = tf(Css);

Gcl = feedback(Gtf,Ctf);
Gbar = Gtf+1/Ctf;

figure
bode(Gtf,Gbar,1/Gbar);
leg1 = legend('$P(s)$','$\bar{G}$','$\frac{1}{\bar{G}(s)}$');
set(leg1,'interpreter','latex','FontSize',12);
>>>>>>> 7478e8fc0ee9bf0e0dfbaf00b289daef5c073baa
