%% PART I
    % Here we test the LMI on a proper and improper stable plant

    numP = [1 -2]; %plant numerator;
    denP = [1 10 100]; %plant denominator (PROPER)
    P = tf(numP,denP);

    G1 = tf([1,1,1],[1,1,4]);
    G2 = tf([1,2],[1,5]);
    P = feedback(G1,G2);
    P = tf([0.5 3.5 4],[1 3 2]);
    % [A, B, C, D, E] = tf2dss(numP,denP);
    [A, B, C, D, E] = ss2dssD(ss(P));
    % 
    % A = [A zeros(size(A,1),1);
    %      zeros(1,size(A,2)+1)];
    % B = [B;0];
    % C = [C 0];
    % E = blkdiag(E,0);


%     E = eye(3); E(3,3)=0;
%     A = eye(3); A(1,1) = -1; A(2,2) = -2;
%     B = ones(3,1);
%     B(3,1) = 0;
%     C = ones(1,3);
%     D = 0.5;

%   another example
    E = eye(3); E(3,3)=0;
    A = -eye(3); A(3,3) = -3; A(2,2) = -2;
    B = [1;0;0];
    C = [1 0 0];
    D = 1;
    
    
    P = tf(dss(A,B,C,D,E));
    [A, B, C, D, E] = ss2dssD(ss(P));
    % [Pdss, A,B,C,D,E] = tf2dss(P);
    % [A,B,C,D,E] = ss2dssD(ss(P));
    % if norm(D)<0.01
    %     [A, B, C, D, E] = ss2dssD(ss(A,B,C,D));
    % end

    n = size(A,1);


    % This subsection is dedicated to trying the first LMI (which supposedly
    % has unwanted numerical properties (equality constraint is problematic)
    eps = 1e-4; % tolerance

    if LMI == 1
    Q = sdpvar(n,n,'full','real');
    % Q = sdpvar(n);


    M1 = [A*Q+(A*Q)',     B-(C*Q)';
            B'-(C*Q),       -(D+D')]; % < 0

    LMI1 = [M1 <= -eps];

    M2 = E*Q; % >=0
    LMI2 = [M2 >= 0];

    t = sdpvar(1);
    M3 = E*Q - Q'*E;
    LMI3 = [M3 >= t]; % equality constraint
    % LMI3 = [M3 == 0];

    % F = [LMI2, LMI3];
    F = [LMI1, LMI2, LMI3, t>=0];

    elseif LMI == 2


    % The other LMI 
    P = sdpvar(n);
    V = null(E);  % a matrix consisting of null space of E
    S = sdpvar(size(V,2), n);

    M1 = [A*(P*E'+V*S)+(P*E'+V*S)'*A',  B-(P*E'+V*S)'*C';
          B'-C*(P*E'+V*S),              -(D+D')];

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

    % LMI1 = double(LMI1)
    % LMI2 = double(LMI2)
    % LMI3 = double(LMI3)
    % 
    % Q = double(Q)
    % 
    % normEQ = norm((E*Q)-(E*Q)')