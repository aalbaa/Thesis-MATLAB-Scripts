% clear all;
% S =[     0     0     0     0     0     0     0     0     0
%      1     0     0     0     0     1     0     0     0
%     -2     1     0     0     0    -1     1     0     0
%      0    -2     1     0     0     0    -1     1     0
%      0     0    -2     1     0     0     0    -1     1
%      0     0     0    -2     1     0     0     0    -1
%      0     0     0     0    -2     0     0     0     0];
% % % 
% % %  
% % % c =[
% % %            1
% % %           21
% % %          175
% % %          735
% % %         1624
% % %         1764
% % %          720];
% %      
% %  degP=3;
% % degQ=4;
% 
% num = [1 -2];
% 
% den=[1 -1 0];


     
A = [];
b=[];
% x0 = S\c;
x0 = ones(7,1);
% x0(1,1) = 1;

options = optimoptions('fmincon');
options.MaxFunctionEvaluations = 10000;
options.StepTolerance = 1e-3;
options.ConstraintTolerance = 1e-3;
func = @(x) norm(S*x-c);
% nonlin = @(x) roots(conv(den,x(1:1:degQ+1)));

Aeq=[];
beq=[];
lb=-Inf*ones(7,1);
ub=inf*ones(7,1);
nonlincon = @(x) nonlin(x,den,degQ);

sol=fmincon(func,x0,A,b,Aeq,beq,lb,ub,nonlincon);



Gp=tf(num,den);


Gc = tf(sol(1:degQ+1)',sol(degQ+2:degQ+2+degP)');
Gfb=feedback(Gp,Gc);
Gbar = Gp+1/Gc;