function [isespr, b, c] = isSPR(SYS, eps, solver)
if nargin == 2    
    solver = 'mosek';
elseif nargin == 1
    eps = 1e-5;
    solver = 'mosek';
end


[A, B, C, D, E] = dssdata(SYS);

if any(any(E ~=eye(size(A))))
    disp('plant must be strictly proper');
    return;
end
 
n = size(A,1);

Q = sdpvar(n);

M1 = [A*Q+(A*Q)',     B-(C*Q)';
        B'-(C*Q),       -(D+D')]; % < 0


LMI1 = [M1 <= -eps];


F = [LMI1];
    
    

opt = sdpsettings;
opt.verbose = 0;

opt.solver = solver;
opt.mosek.MSK_DPAR_ANA_SOL_INFEAS_TOL = 1e-3;


sol=solvesdp(F,[],opt);

a = double(LMI1);


isespr = a;
end