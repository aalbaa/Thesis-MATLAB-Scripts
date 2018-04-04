% this function is to determine whether a plant is ESPR or not

function [isespr, b, c] = isESPR(SYS, eps, solver)
if nargin == 2    
    solver = 'mosek';
elseif nargin == 1
    eps = 1e-5;
    solver = 'mosek';
end


[A, B, C, D, E] = dssdata(SYS);

 
n = size(A,1);

Q = sdpvar(n);

M1 = [A*Q+(A*Q)',     B-(C*Q)';
        B'-(C*Q),       -(D+D')]; % < 0


LMI1 = [M1 <= -eps];


M2 = E*Q; % >=0
LMI2 = [M2 >= 0];

M3 = E*Q - Q'*E;
LMI3 = [M3 == 0]; % equality constraint

F = [LMI1, LMI2, LMI3];
    
    

opt = sdpsettings;
opt.verbose = 0;

opt.solver = solver;
opt.mosek.MSK_DPAR_ANA_SOL_INFEAS_TOL = 1e-3;


sol=solvesdp(F,[],opt);

a = double(LMI1);
b = double(LMI2);
c = double(LMI3);

isespr = a&&b&&all(all(c));
end
    