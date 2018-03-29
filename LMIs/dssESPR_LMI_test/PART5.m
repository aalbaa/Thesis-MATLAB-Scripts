% This part is directly involved with finding a controller of a plant uing
% the KYP lemma (State Space)

% This is just constructing the TFs
 num = [0.5 3.5 4];  % numerator of the passive system
    den = [1 3 2];      % denominator of the passive system (example from Part I)
    rts = roots(num);   % roots of the numerator
    bbar = -rts(1);      % b(s) = s + bbar;
    pbar = -rts(2);      % p(s) = s + pbar;

    % let   a(s) := a0 + a1*s;
    % and   q(s) := q0 + q1*s;
    % Ax = b; where x = [a1, a0, q1, q0]';

    A_le = [1, 0, 1, 0;
         pbar, 1, bbar, 1;
         0, pbar, 0, bbar];
    b_le = den';

    x = A_le\b_le;

    as = x(1:2);        % a(s)
    qs = x(3:4);        % q(s)

    Ptf = tf([1 bbar],as');
    Ctf = tf(qs', [1 pbar]);

    FB = feedback(Ptf,Ctf);
% End of constructing controllers
    
% Ptf = tf([1 1],[1 2 3]);


[Ap,Bp,Cp,Dp] = ssdata(Ptf);


np = size(Ap,1);
nc = np;

Q1 = sdpvar(np);
Q2 = sdpvar(np,nc,'full');
Q3 = Q2';
% Q4 = sdpvar(nc);
Q4 = Q3;

Ac = sdpvar(nc,nc,'full');
% Bc = sdpvar(nc,1);
Bc = ones(nc,1);
Cc = sdpvar(1,nc);
Dc = sdpvar(1);

Ccbar = sdpvar(1,np);
Acbar = sdpvar(nc,np);

    AclQ = [Ap*Q1-Bp*Ccbar,  Ap*Q2-Bp*Ccbar;
                    Bc*Cp*Q1+Acbar-Bc*Dp*Ccbar, Bc*Cp*Q2+Acbar-Bc*Dp*Ccbar];
    Bcl = [Bp; Bc*Dp];
    CclQ = [Cp*Q1-Dp*Cc*Q3,   Cp*Q2-Dp*Cc*Q4];
    Dcl = Dp;
%         Ecl = blkdiag(Epdss,Ecdss);
    
    

    M1 = [AclQ+AclQ',   Bcl-(CclQ)';
                    Bcl'-CclQ,  -(Dcl+Dcl')];
    
    Q = [Q1,Q2;Q3,Q4];
          
    eps1 = 1e-4;
    
    LMI1 = [Q >= eps1];
    LMI2 = [M1 <= -eps1];
    
    F = [LMI1, LMI2];
    
    t = [];