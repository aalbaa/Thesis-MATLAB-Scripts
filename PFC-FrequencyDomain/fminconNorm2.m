S =[     0     0     0     0     0     0     0     0     0
     1     0     0     0     0     1     0     0     0
    -2     1     0     0     0    -1     1     0     0
     0    -2     1     0     0     0    -1     1     0
     0     0    -2     1     0     0     0    -1     1
     0     0     0    -2     1     0     0     0    -1
     0     0     0     0    -2     0     0     0     0];

 
c =[
           1
          21
         175
         735
        1624
        1764
         720];
     
 degP=3;
degQ=4;

num = [1 -2];

den=[1 -1 0];


     
A = [];
b=[];
x0 = ones(9,1);


func = @(x) norm(S*x-c)^2;
% nonlin = @(x) roots(conv(den,x(1:1:degQ+1)));

Aeq=[];
beq=[];
lb=-Inf*x0;
ub=inf*x0;

sol=fmincon(func,x0,A,b,Aeq,beq,lb,ub,nonlin);



Gp=tf(num,den);


Gc = tf(sol(1:degQ+1)',sol(degQ+2:degQ+2+degP)');
Gfb=feedback(Gp,Gc);
Gbar = Gp+1/Gc;