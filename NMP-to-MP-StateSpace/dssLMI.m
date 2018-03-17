%
clear all;
close all;
home;

g = tf([1], [1 -1 1]);
G = ss(g);
p = size(G.A,1);
Ap = [G.A, zeros(p,1);
        zeros(1,p),  1];
    
Bp = [G.B; 1];
Cp = [G.C, 1];
Dp = G.D+1;
Ep = eye(p+1); Ep(p+1,p+1)=0;

%Say we want our controller to be 2 var
n = 3;
% Ec = eye(n); Ec(n,n)=1;

Dc = 0;

Ac = sdpvar(n,n,'full','real');
% Bc = sdpvar(n,1,'full','real');
% Bc = [ones(n,1)];
% Bc = [zeros(n-1,1); 1];
Bc = [8.3083
    5.8526
    5.4972];

% Bc = rand(n,1)*10;
Cc = sdpvar(1,n,'full','real');


% Acl = [Ap, -Bp*Cc;
%         Bc*Cp,  Ac];
Bcl = [Bp; Bc*Dp];   


% Ccl = [Cp, zeros(1,n)];     % Mistake
% Ccl = [Cp, -Dp*Cc];       % Corrected

Dcl = Dp;

% Ecl = blkdiag(Ep,Ec);

% dimCL = size(Acl,1);

% Q = sdpvar(dimCL);
Q1 = sdpvar(p+1,p+1,'full','real');
Q2 = sdpvar(p+1,p+1,'full','real');
Q = [Q1 Q2;Q2 Q2];

Ccbar = sdpvar(1,n, 'full','real');
Acbar = sdpvar(n,n,'full','real');

Ecbar = sdpvar(n,n,'full','real');


% AclQ = [Ap*Q1-Bp*Ccbar, Ap-Bp*Ccbar;
%         Bc*Cp*Q1+Acbar, Bc*Cp*Q2+Acbar];      % It has some mistakes

AclQ = [Ap*Q1-Bp*Ccbar,                 Ap*Q2-Bp*Ccbar;
        Bc*Cp*Q1+Acbar-Bc*Dp*Ccbar,     Bc*Cp*Q2+Acbar+ Bc*Dp*Ccbar]; % Corrected
    
CclQ = [Cp*Q1-Dp*Ccbar,     Cp*Q2-Dp*Ccbar];

EclQ = [Ep*Q1 Ep*Q2;
        Ecbar,  Ecbar];

    
    
eps1 = 1e-2;

% LMI = [AclQ+AclQ', Bcl-([Cp*Q1 zeros(1,n)])';
%    Bcl'-([Cp*Q1, zeros(1,n)]),   -(Dcl+Dcl')]; % Old (Cc doesn't have quadratic terms)

LMI = [ AclQ+(AclQ)',   Bcl-(CclQ)';
        Bcl'-CclQ,      -(Dcl+Dcl')];
    
LMI1 = [LMI <= -eps1];%*eye(size(LMI))];
LMI2 = [EclQ == EclQ'];
LMI3 = [EclQ >=0];% zeros(size(EclQ))];
LMI4 = [Q2+Q2' >= eps1*1];%eye(size(Q2))];



% F = [LMI2, LMI1, LMI3];
% F = [LMI1, LMI2, LMI3];
F = [LMI2, LMI1];
% F = LMI3;
%LMI3 is the cause of infeasibility


% opt = sdpsettings('solver','mosek','verbose',0);
% opt = sdpsettings('solver','sdpt3','verbose',0);
opt = sdpsettings('solver','sedumi','sedumi.eps',0);
% opt = [];


solvesdp(F,[],opt);
% solvesdp(F);


Q1 = double(Q1);
Q2 = double(Q2);
Ecbar = double(Ecbar);
Acbar = double(Acbar);
Ccbar = double(Ccbar);

Q2inv = inv(Q2);
Ec = Ecbar*Q2inv;
Ac = Acbar*Q2inv;
Cc = Ccbar*Q2inv;

Gc = dss(Ac,Bc,Cc,Dc,Ec);
C = tf(Gc);


% Acl = [Ap, -Bp*Cc;
%         Bc*Cp,  Ac-Bc*Dp*Cc];
%     
% Bcl = [Bp; Bc*Dp];
% 
% Ccl = [Cp, -Dp*Cc];
% 
% Dcl = Dp;
% 
% Ecl = blkdiag(Ep,Ec);


[Acl, Bcl, Ccl, Dcl, Ecl] = dss2dssCL(G,Gc);
Gcl = dss(Acl,Bcl,Ccl,Dcl,Ecl);
TFcl = tf(Gcl);


% 
% LMI1 = [Acl1*Q+(Acl1*Q)'+P+P', Bcl-(Ccl*Q)';
%         (Bcl-(Ccl*Q)')',    -(Dcl+Dcl')];
% LMI1 = [LMI1 <= -eps*eye(size(LMI1,1))];
% 
% LMI2 = [Ecl*Q == (Ecl*Q)'];
% 
% LMI3 = [Ecl*Q >= 0];
% LMI4 = [Q >= eps*eye(size(Q))];
% 
% F = [LMI1, LMI2, LMI3, LMI4];
% 
% opt = sdpsettings('solver','mosek','verbose',0);
% solvesdp(F,[],opt)
% 
% P = double(P);
% Q = double(Q);
% 
% Acl2 = inv(Q)*P;
% 
% 
% Ac = double(Ac);
% Bc = double(Bc);
% Cc = double(Cc);
% Ec = double(Ec);
% 
% 
% 
% Acl = [Ap, -Bp*Cc;
%         Bc*Cp,  Ac];
% Bcl = [Bp; Bc*Dp];
% 
% Ccl = [Cp, zeros(1,n)];
% 
% Dcl = Dp;
% 
% Ecl = blkdiag(Ep,Ec);

% 
% Acl212 = Acl2(1:3,p+2:size(Acl2,2));
% Acl221 = Acl2(p+2:size(Acl2,1),1:size(Ap,1));
% Acl222 = Acl2(p+2:size(Acl2,1),size(Ap,1)+1:size(Acl2,2));
% Acl212 = -Acl212;





% Gcl = dss(Acl,Bcl,Ccl,Dcl,Ecl);
% gcl = tf(Gcl);
% pole(gcl)

