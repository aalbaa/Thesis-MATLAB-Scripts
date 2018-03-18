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

PlantNumerator = [1 2];
PlantDenominator = [1 -1 1];
Gtf = tf(PlantNumerator,PlantDenominator);

% state space
[Ag Bg Cg Dg] = tf2ss(PlantNumerator,PlantDenominator);

% Constructing the regulator matrices

A = Ag;

B1 = zeros(size(A,1),wSize);   
B2 = Bg;

C1 = [-Cg;zeros(size(Cg,1),size(A,2))];
C2 = -Cg;

D11 = [ones(size(Cg,1),wSize); zeros(size(Cg,1),wSize)];
D12 = [-Dg;ones(size(Cg,1),size(Bg,2))];
D21 = ones(size(Cg,1),wSize);
D22 = -Dg;

%constructing P (for optimal control formulation)

P = ss(Ag,[B1 B2], [C1;C2], [D11 D12; D21 D22]);

%finding the optimal controller K, and the closed loop ss CL

[K CL] = h2syn(P,size(B2,2),size(Cg,1));


%controller transfer function
[Cnum, Cden] = ss2tf(K.A,K.B,K.C,K.D);
Ctf = tf(Cnum,Cden);


%closed loop
Gcl = feedback(Gtf,Ctf);

disp("Poles of Gcl: ");
disp(pole(Gcl));

disp("Poles of CL (ss): ");
disp(eig(CL.A));


