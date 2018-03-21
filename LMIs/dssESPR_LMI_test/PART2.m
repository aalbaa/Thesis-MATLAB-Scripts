
    %% PART II
    % The second part consists of testing the LMI on a system with its

    % Idea: Develop a Plant P, and a controller C, such that the feedback
    % (feedback(P,C) results in a ESPR system (we use the ESPR example in part
    % I to developo the P and C)

    %Let P = b/a; C = q/p,
    % then FB := feedback(P,C) = bp/(ap+qb).
    % so zeros(FB) = zeros(P)+poles(C);
    
% SS = 0; % = 1 if you want to try the State Space; 
        % = 0 if you want to try the transfer function
if SS == 0
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


    P = FB;

    [Acl, Bcl, Ccl, Dcl, Ecl] = ss2dssD(ss(P),1);

    n = size(Acl,1);

    % This subsection is dedicated to trying the first LMI (which supposedly
    % has unwanted numerical properties (equality constraint is problematic)

    eps = 1e-4; % tolerance

    if LMI == 1
        Q = sdpvar(n,n,'full','real');
        % Q = sdpvar(n);



        M1 = [Acl*Q+(Acl*Q)',     Bcl-(Ccl*Q)';
                Bcl'-(Ccl*Q),       -(Dcl+Dcl')]; % < 0

        LMI1 = [M1 <= -eps];

        M2 = Ecl*Q; % >=0
        LMI2 = [M2 >= 0];

        t = sdpvar(1);
        M3 = Ecl*Q - Q'*Ecl;
        LMI3 = [M3 <= t]; % equality constraint
        % LMI3 = [M3 == 0];

        % F = [LMI2, LMI3];
        F = [LMI1, LMI2, LMI3, t>=0];

    elseif LMI == 2   



        % The other LMI 
        P = sdpvar(n);
        V = null(Ecl);  % a matrix consisting of null space of E
        S = sdpvar(size(V,2), n);

        M1 = [Acl*(P*Ecl'+V*S)+(P*Ecl'+V*S)'*Acl',  Bcl-(P*Ecl'+V*S)'*Ccl';
              Bcl'-Ccl*(P*Ecl'+V*S),              -(Dcl+Dcl')];

        LMI1 = [M1 <= -eps];
        LMI2 = [P >= eps];


        F = [LMI1, LMI2];
        
        t = [];

    end

    opt = sdpsettings('solver','mosek','verbose',1);
    % opt = sdpsettings('solver','sdpt3','verbose',1);
    % opt = sdpsettings('solver','sedumi','verbose',1);
    % opt = [];


    solvesdp(F,t,opt);
%     solvesdp(F,[],opt);


% in DSS 
elseif SS == 1    
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

        %%%%%%
        %%%%%%
    Ptf = tf([1 bbar],as');
    Ctf = tf(qs', [1 pbar]);    
    
    [Pdss, Apdss, Bpdss, Cpdss, Dpdss, Epdss] = tf2dss(Ptf);
    [Cdss, Acdss, Bcdss, Ccdss, Dcdss, Ecdss] = tf2dss(Ctf);
    
    [Acdss, Bcdss, Ccdss, Dcdss, Ecdss, CdssD] = ss2dssD(Cdss,0); 
    [Apdss, Bpdss, Cpdss, Dpdss, Epdss, PdssD] = ss2dssD(Pdss,1); 
    % this is converting the Cdss to Cdss2 with ZERO D
    % and ensuring that Dpdss != 0 so the conditions are met
    
    
    
    % ensure D = 0
    Acl = [Apdss, -Bpdss*Ccdss;
            Bcdss*Cpdss,    Acdss-Bcdss*Dpdss*Ccdss];
    Bcl = [Bpdss; Bcdss*Dpdss];
    Ccl = [Cpdss, -Dpdss*Ccdss];
    Dcl = Dpdss;
    Ecl = blkdiag(Epdss,Ecdss);


    n = size(Acl,1);

    
    % This subsection is dedicated to trying the first LMI (which supposedly
    % has unwanted numerical properties (equality constraint is problematic)

    eps = 1e-4; % tolerance

    if LMI == 1
        Q = sdpvar(n,n,'full','real');
        % Q = sdpvar(n);



        M1 = [Acl*Q+(Acl*Q)',     Bcl-(Ccl*Q)';
                Bcl'-(Ccl*Q),       -(Dcl+Dcl')]; % < 0

        LMI1 = [M1 <= -eps];

        M2 = Ecl*Q; % >=0
        LMI2 = [M2 >= 0];

        t = sdpvar(1);
        M3 = Ecl*Q - Q'*Ecl;
        LMI3 = [M3 <= t]; % equality constraint
        % LMI3 = [M3 == 0];

        % F = [LMI2, LMI3];
        F = [LMI1, LMI2, LMI3, t>=0];

    elseif LMI == 2   



        % The other LMI 
        P = sdpvar(n);
        V = null(Ecl);  % a matrix consisting of null space of E
        S = sdpvar(size(V,2), n);

        M1 = [Acl*(P*Ecl'+V*S)+(P*Ecl'+V*S)'*Acl',  Bcl-(P*Ecl'+V*S)'*Ccl';
              Bcl'-Ccl*(P*Ecl'+V*S),              -(Dcl+Dcl')];

        
        LMI1 = [M1 <= -eps];
        LMI2 = [P >= eps];


        F = [LMI1, LMI2];
        
        t = [];

    end
    
    if solver == 1
        opt = sdpsettings('solver','mosek','verbose',1);
    elseif solver == 2       
        opt = sdpsettings('solver','sdpt3','verbose',1);
    elseif solver == 3        
        opt = sdpsettings('solver','sedumi','verbose',1);
    else
        opt = [];
    end


    solvesdp(F,t,opt);
%     solvesdp(F,[],opt);
    
end