% Amro AL Baali
% March 17, 2018
%
% This is to test the DSS LMI (there exists a Q s.t the LMI holds)
%
% PART I: The first part consists of testing the LMI on one stable system

% PART II: The second part consists of testing the LMI on a system with its
% controller using the CL equations developed by hand

% PART III: Solve LMI to FIND a controller

clear all;
close all;
home;


% in = input('[part, LMI, solver]\n');
% part = in(1);
% LMI = in(2);
% solver = in(3);
part = 3;   % 1, 2, 3
LMI = 1; % 1: KYP, 2: ESPR LMI1 (with equality), 3: ESPR LMI2 (No equality)
solver = 1; % 1: mosek. 2: sdpt3. 3: sedumi. anything else: no specific solve
SS = 1;
verb = 1;
eps = 1e-4;

% becomes 1 if Ac and Cc are replaced by Acbar=Ac*Q3 and Ccbar=Cc*Q3
invert = 0; 
% 
if part == 1
    PART1;
elseif part == 2
    PART2;
elseif part == 3
    PART3;
elseif part == 4
    PART4;
elseif part == 5
    PART5;
end

opt = sdpsettings;
opt.verbose = verb;
opt.removeequalities = [1 (0)];
% solver = 2;

if solver == 1    
    opt.solver = 'mosek';
    opt.mosek.MSK_DPAR_ANA_SOL_INFEAS_TOL = 1e-3;
elseif solver == 2       
    opt.solver = 'sdpt3';
%     opt.sedumi.numtol = 1e-4;
%     opt.sedumi.eps = 1e-5;
%     opt.sedumi.bigeps = 1e-2;
%     opt.sedumi.bignumtol = 1;
elseif solver == 3        
    opt.solver = 'sedumi';
else
    opt = [];
end


sol=solvesdp(F,t,opt);
%     solvesdp(F,[],opt);


if invert == 1
    Q3 = double(Q3);
    Ac = double(Acbar)*inv(Q3);
    Cc = double(Ccbar)*inv(Q3);
end

Bc = double(Bcdss);
% Cc = double(Ccdss);
Dc = double(Dcdss);
Ec = double(Ecdss);


t = double(t);