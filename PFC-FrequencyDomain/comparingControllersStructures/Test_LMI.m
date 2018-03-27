%%
%Amro Al Baali
%Jan 23, 2018


%Trying to find a controller for Gp3 that makes the augmented plant (Gbar)
%nonminimum phase AND STABLE!
%It involved solving the RH table by hand


%NOTE: this is valid for Gp3 and of a controller of degree 1/1 ONLY! didn't
%analyze if it can work on general plants
%% %%%%%%%%%%%%%
% C = q(s)/p(s), S*X'=c;
%X = [q1 q0 p1 p0];
clear all;
home;

TFs;
degP = 1;
degQ = 2;
Gp = Gp4;

[S c Tq Tp]=sylvester(Gp,degP,degQ); %equality contraint

%% LMI
alpha  = 10; %from Gp3 = (s-b)/(
F0 = [alpha^2 alpha^(3/2) alpha;
    alpha^(3/2) 0 0;
    alpha 0 0];
F1 = zeros(3,3); F1(3,3)=1; %associated with q1
F2 = zeros(3,3); F2(2,2)=1; %associated with q0
F3 =zeros(3,3); %associated with p1 (nothing)
F4 = zeros(3,3);%associated with p0 (nothing)

%X = [q1 q0 p1 p0];
X = sdpvar(degP+degQ+2,1);
M1 = F0+X(1)*F1+X(2)*F2; %>0
M2 = X(1); %q1>0 from RH table
M3 = X(2)-X(1)*alpha; %q0-alpha*q1>0;
eps1 = 1e-10;
% LMI = [M1>eps1,  M2>eps1, M3 > eps1, S*X==c];

M1 = [S*X==c];
denPlant = Gp.denominator{1,1};
numPlant = Gp.Numerator{1,1};
% degP = 1;
T = toeplitz([denPlant zeros(1,degQ)],[denPlant(1) zeros(1,degQ)]);;
M2 = T*X(1:degQ+1);
M3 = M2(2)*M2(5)-M2(3)*M2(4);
M4 = M2(2)*M3-M2(1)*M2(4);
% M5 = rhStabilityCriterion(M2);
% LMI = [M3>0.1,M4>0.1, M2(1)>0.1,M2(4)>0.1];
% c2 = poly([-15*rand(1)*rand(size(T,1)-1,1)])';
% M6 = nonlconLMI(X,numPlant, denPlant, degP, degQ);


%% nonlconLMI

q = X(1:degQ+1);
p = X(degQ+2:degQ+2+degP);

% convolution of numerator
num1 = conv(numPlant,q);
num2 = conv(denPlant,p);

GbarDen = conv(denPlant,q);

% cDen = rhStabilityCriterion(GbarDen');

cDen = real(roots(GbarDen'));
M7 = cDen;





LMI = [M1];
% LMI = [M5>0.1];

% LMI = [M1>eps1,  M2>eps1, M3 > eps1];
% options = sdpsettings('solver','sedumi','sedumi.eps',1e-10,'verbose',1);
solvesdp(LMI);

x = double(X)';
% 
C = tf(x(1:degQ+1),x(degQ+2:degQ+degP+2));
Gbar = Gp+1/C;