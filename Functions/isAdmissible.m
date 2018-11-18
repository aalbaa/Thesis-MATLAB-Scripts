% This script is to test the admissibility of (E,A)
function [isadmissible] = isAdmissible(SYS, solver, eps, verb)

% SYS = tf([1 1],[1 2 1]);
% eps = 1e-5;
% solver = 1;

if nargin <= 3
    verb = 0;
    if nargin <= 2
        eps = 1e-5;
        if nargin == 1            
            solver = 1;
        end
    end
end

    

[A, B, C, D, E] = dssdata(SYS);

n = size(A,1);

P = sdpvar(n);

X = sdpvar(n);
Y = sdpvar(n);

LMI1 =[E'*X*A+A'*X*E == -E'*Y*E];
LMI2 = [Y >= eps];
LMI3 = [X >= eps];

F = [LMI1, LMI2, LMI3];
t = [];

%% Solving the sdp



% solver = 1;
% verb = 1;

opt = sdpsettings;
opt.verbose = verb;
% opt.removeequalities = 1;

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


sol=solvesdp(F,t,opt);

% a = double(LMI1);
% b = double(LMI2);
% c = double(LMI3);

isadmissible = ~isempty(findstr(sol.info,'Successfully'));
end
