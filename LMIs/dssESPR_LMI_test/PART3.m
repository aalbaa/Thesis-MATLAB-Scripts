% controller using the CL equations developed by hand
%% PART III
% Here we use the Plant developed in PART II and we want to obtain a
% controller for that Plant that renders the CL ESPR. We know such
% controller exists because we synthesized the Plant in such a way.


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
    [Apdss, Bpdss, Cpdss, Dpdss, Epdss, PdssD] = ss2dssD(Pdss,1); 
    
    
    % now we want to find such a plant
    % we'll assume that we have Ecdss
    [Cdss, Acdss, Bcdss, Ccdss, Dcdss, Ecdss] = tf2dss(Ctf);    
    [Acdss, Bcdss, Ccdss, Dcdss, Ecdss, CdssD] = ss2dssD(Cdss,0); 
    
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

    eps = 1e-2; % tolerance

    if LMI == 1
         np = size(Apdss,1);
        nc = size(Bcdss,1);
        
        Ecdss = [Ecdss, zeros(nc,1);
                 zeros(1,nc+1)]; Ecdss(3,3) = 1;
        Acdss = [Acdss, zeros(nc,np-nc);
                zeros(np-nc,np)]; Acdss(3,3) = 1;
        Bcdss = [Bcdss;zeros(np-nc,1)];
        Ccdss = [Ccdss, zeros(1,np-nc)];
        %thus
        nc = np;
        
        
        
        Acbar = sdpvar(nc,nc,'full');
        Ccbar = sdpvar(1,nc);
        invert = 1; % to notify 'main' that we're using Acbar and Ccbar
        
        %size of Bc must be the same as size of Bp (look at Q2)
        
        Q1 = sdpvar(np);
%         Q1 = sdpvar(np,np,'full','real');
        Q2 = sdpvar(np,nc,'full','real');     
%         Q3 = sdpvar(nc,np,'full','real');
        Q3 = Q2';
%         Q4 = sdpvar(nc,nc,'full','real');
%         Q4 = sdpvar(nc);
        Q4 = Q3;
        
        Q = [Q1, Q2; Q3, Q4];
        
        AclQ = [Apdss*Q1-Bpdss*Ccbar,  Apdss*Q2-Bpdss*Ccbar;
                Bcdss*Cpdss*Q1+Acbar-Bcdss*Dpdss*Ccbar, Bcdss*Cpdss*Q2+Acbar-Bcdss*Dpdss*Ccbar];
        Bcl = [Bpdss; Bcdss*Dpdss];
        CclQ = [Cpdss*Q1-Dpdss*Ccdss*Q3,   Cpdss*Q2-Dpdss*Ccdss*Q4];
        Dcl = Dpdss;
        Ecl = blkdiag(Epdss,Ecdss);
        
        M1 = [AclQ+AclQ',   Bcl-(CclQ)';
                Bcl'-CclQ,  -(Dcl+Dcl')];
        
        
        LMI1 = [M1 <= -eps];
        
        t = [];
        
        F = [LMI1];        
        
    elseif LMI == 2
        
        %we want Q3 to be square, so size Ac must equal size Ap
        
        
        np = size(Apdss,1);
        nc = size(Bcdss,1);
        
        Ecdss = [Ecdss, zeros(nc,1);
                 zeros(1,nc+1)]; Ecdss(3,3) = 1;
        Acdss = [Acdss, zeros(nc,np-nc);
                zeros(np-nc,np)]; Acdss(3,3) = 1;
        Bcdss = [Bcdss;zeros(np-nc,1)];
        Ccdss = [Ccdss, zeros(1,np-nc)];
        %thus
        nc = np;
        
        % does min realization affect things?
%         PminSys = minreal(dss(Apdss,Bpdss,Cpdss,Dpdss,Epdss));
%         CminSys = minreal(dss(Acdss,Bcdss,Ccdss,Dcdss,Ecdss));
%         
%         Apdss = PminSys.A;
%         Bpdss = PminSys.B;
%         Cpdss = PminSys.C;
%         Dpdss = PminSys.D;
%         Epdss = PminSys.E;
%         if isempty(Epdss)
%             Epdss=eye(size(Apdss,1));
%         end
%         
%         np = size(Apdss,1);
%         
%         Acdss = CminSys.A;
%         Bcdss = CminSys.B;
%         Ccdss = CminSys.C;
%         Dcdss = CminSys.D;
%         Ecdss = CminSys.E;
%         if isempty(Ecdss)
%             Ecdss=eye(size(Acdss));
%         end
%         
%         
%         nc = size(Acdss,1);
        
        Acbar = sdpvar(nc,nc,'full');
        Ccbar = sdpvar(1,nc);
        
        %size of Bc must be the same as size of Bp (look at Q2)
        
        Q1 = sdpvar(np);
%         Q1 = sdpvar(np,np,'full','real');
        Q2 = sdpvar(np,nc,'full','real');     
%         Q3 = sdpvar(nc,np,'full','real');
        Q3 = Q2';
%         Q4 = sdpvar(nc,nc,'full','real');
%         Q4 = sdpvar(nc);
        Q4 = Q3;
        
        Q = [Q1, Q2; Q3, Q4];
        
        AclQ = [Apdss*Q1-Bpdss*Ccdss*Q3,  Apdss*Q2-Bpdss*Ccdss*Q4;
                Bcdss*Cpdss*Q1+Acdss*Q3-Bcdss*Dpdss*Ccdss*Q3, Bcdss*Cpdss*Q2+Acdss*Q4-Bcdss*Dpdss*Ccdss*Q4];
        Bcl = [Bpdss; Bcdss*Dpdss];
        CclQ = [Cpdss*Q1-Dpdss*Ccdss*Q3,   Cpdss*Q2-Dpdss*Ccdss*Q4];
        Dcl = Dpdss;
        Ecl = blkdiag(Epdss,Ecdss);
        
        M1 = [AclQ+AclQ',   Bcl-(CclQ)';
                Bcl'-CclQ,  -(Dcl+Dcl')];

        LMI1 = [M1 <= -eps];

        M2 = Ecl*Q; % >=0
        LMI2 = [M2 >= 0];

        
        M3 = Ecl*Q - Q'*Ecl;
        LMI3 = [M3 <= t]; % equality constraint
        % LMI3 = [M3 == 0];

        % F = [LMI2, LMI3];
        F = [LMI1, LMI2, LMI3, t>=0];

    elseif LMI == 3   


         
        np = size(Apdss,1);
        nc = size(Bcdss,1);
        
        Ecdss = [Ecdss, zeros(nc,1);
                 zeros(1,nc+1)]; Ecdss(3,3) = 1;
        Acdss = [Acdss, zeros(nc,np-nc);
                zeros(np-nc,np)]; Acdss(3,3) = 1;
        Bcdss = [Bcdss;zeros(np-nc,1)];
        Ccdss = [Ccdss, zeros(1,np-nc)];
        %thus
        nc = np;
        
        % The other LMI 

        Acbar = sdpvar(nc,nc,'full');
        Ccbar = sdpvar(1,nc);
        
%         Q1 = sdpvar(np,np,'full','real');
        Q1 = sdpvar(np);
%         Q2 = sdpvar(np,nc,'full','real');  
        Q2 = sdpvar(np);
        Q3 = Q2';        
%         Q3 = sdpvar(nc,np,'full','real');        
%         Q4 = sdpvar(nc,nc,'full','real');
        Q4 = Q3;
        
        Q = [Q1, Q2; Q3, Q4];
        
        Acl = [Apdss, -Bpdss*Ccdss;
            Bcdss*Cpdss,    Acdss-Bcdss*Dpdss*Ccdss];
        Ccl = [Cpdss, -Dpdss*Ccdss];
        
        AclQ = [Apdss*Q1-Bpdss*Ccbar,  Apdss*Q2-Bpdss*Ccbar;
                Bcdss*Cpdss*Q1+Acbar-Bcdss*Dpdss*Ccbar, Bcdss*Cpdss*Q2+Acbar-Bcdss*Dpdss*Ccbar];
        Bcl = [Bpdss; Bcdss*Dpdss];
        CclQ = [Cpdss*Q1-Dpdss*Ccbar,   Cpdss*Q2-Dpdss*Ccbar];
        Dcl = Dpdss;
        Ecl = blkdiag(Epdss,Ecdss);
        
        P = sdpvar(np+nc);
        V = null(Ecl);  % a matrix consisting of null space of E
        S = sdpvar(size(V,2), np+nc);
        
        M1 = [AclQ*Ecl'+Acl*V*S+Ecl*AclQ'+S'*V'*Acl',   Bcl-Ecl*(CclQ)'-S'*V'*Ccl';
                Bcl'-(CclQ)*Ecl'-Ccl*V*S,  -(Dcl+Dcl')]
                    
        Q = [Q1, Q2; Q2', Q3];
        LMI1 = [M1 <= -eps];
        LMI2 = [Q >= eps];       
        
        F = [LMI1, LMI2];
        
        t = [];

    end
    
%     if solver == 1
%         opt = sdpsettings('solver','mosek','verbose',verb);
% %         opt.beeponproblem=[17:-1:-9];
% %         opt.showprogress=1;
%         opt.removeequalities = 1;
%         
%         opt.debug=1;
%     elseif solver == 2       
%         opt = sdpsettings('solver','sdpt3','verbose',verb);
%     elseif solver == 3        
%         opt = sdpsettings('solver','sedumi','verbose',verb);
%     else
%         opt = [];
%     end
% 
% 
%     solvesdp(F,t,opt);
%     solvesdp(F,[],opt);
    