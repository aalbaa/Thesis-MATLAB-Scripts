% Amro Al Baali
% February 13, 2018
%
%
% Example Of using h2syn(P,sizeB2,sizeC2)
%   Optimal Control formulation:
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

% solver params
solver = 1;
verb = 1;
eps = 1e-5;






%number of exegenous inputs
wSize = 1;
uSize = 1;

%Transfer function used

PlantNumerator = [1 -2];
% PlantDenominator = [1 -1 1];
PlantDenominator = [1 10 100];
Ptf = tf(PlantNumerator,PlantDenominator);

% state space
[Ap Bp Cp Dp] = ssdata(Ptf);



%%%%%%%% Optimal control params

A = Ap;

B1 = Bp;   
% B2 = zeros(size(Bp));
B2 = -Bp;

% C1 = [-Cp;zeros(size(Cp,1),size(A,2))];
C1 = -Cp;
C1 = zeros(1,size(A,2));
% C1 = zeros(1,size(A,2));
C2 = Cp;

D11 = 1; D12 = -1; D21 = 1; D22 = 0;
% D11 = [ones(size(Cp,1),wSize); zeros(size(Cp,1),wSize)];
% D12 = [-Dp;ones(size(Cp,1),size(Bp,2))];
% D21 = ones(size(Cp,1),wSize);
% D22 = -Dp;

%%%%%%%%%%%%



% Constructing the regulator matrices


X1 = sdpvar(size(A,1));
X2 = sdpvar(size(A,1));
Y1 = sdpvar(size(A,1));
Y2 = sdpvar(size(A,1));
Z = sdpvar(size(C1,1));
gamma = sdpvar(1,1);


nc = size(Ap,1);

An = sdpvar(nc,nc,'full');
Bn = sdpvar(nc,1);
Cn = sdpvar(1,nc);
Dn = sdpvar(1,1);





%%% LMI 1;
E11 = A*Y1+Y1*A'+B2*Cn + Cn'*B2';
E12 = A+An'+B2*Dn*C2;
E13 = B1+B2*Dn*D21;
E22 = X1*A+A'*X1+Bn*C2+C2'*Bn';
E23 = X1*B1+Bn+Bn*D21;

LMI1 = [[E11 E12 E13;
        E12' E22 E23;
        E13' E23' -1]<= -eps];
    
%%% LMI 2;
H11 = Y1;
H12 = eye(size(Y1,1),size(X1,2));
H13 = Y1'*C1'+Cn'*D12';
H22 = X1;
H23 = C1'+C2'*Dn'*D12';
H33 = Z;

LMI2 = [[H11 H12 H13;
        H12' H22 H23;
        H13' H23' H33] >= eps];

%%% LMI 3;

LMI3 = [D11+D12*Dn*D21 == 0];

%%% LMI 4;

LMI4 = [ [X1, eye(size(X1,1),size(Y1,2));
          eye(size(X1,1),size(Y1,2)),  Y1] >= eps];

%%% LMI 5;

LMI5 = [trace(Z) <= gamma - eps];       

    

opt = sdpsettings;
opt.verbose = verb;
% opt.removeequalities = [1 (0)];


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

F = [LMI1, LMI2, LMI3, LMI4, LMI5];

sol=solvesdp(F,gamma,opt);

X1 = double(X1);
Y1= double(Y1);

An = double(An);
Bn = double(Bn);
Cn = double(Cn);
Dn = double(Dn);

[X2, Y2] = lu(eye(size(X1*Y1)) - X1*Y1);

% left matrix to obtain K matrices (Ak, Bk, Ck, Dk)
ML = [ X2, X1*B2;
       zeros(1,size(X1,1)), 1];
% right matrix
MR = [Y2', zeros(size(Y2,1),1);
     C2*Y1, 1];
 
K = inv(ML)*([An, Bn; Cn, Dn]-[X1*A*Y1, zeros(size(Bn));zeros(1,size(Cn,2)+1)])*inv(MR);

Ak = K(1:size(An,1),1:size(An,1));
Bk = K(1:size(An,1), size(An,2)+1:end);
Ck = K(size(An,1)+1:end,1:size(An,1));
Dk = K(end,end);

Dc = inv(1+Dk*D22)*Dk;
Cc = (1-Dc*D22)*Ck;
Bc = Bk*(1-Dc*D22);
Ac = Ak-Bc*inv(1-D22*Dc)*D22*Cc;

Gc = ss(Ac,Bc,Cc,Dc);
Ctf = tf(Gc);

Gbar = Ptf+1/Ctf;
Gcl = feedback(Ptf,Ctf);
        

