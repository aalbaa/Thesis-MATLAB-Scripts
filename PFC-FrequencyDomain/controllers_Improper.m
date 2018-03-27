%%%% imporper controllers
%I used pole placement to find an improper controller
%%
%%%%%   FF controller with rel. degree 1 for PLANT 1
%To find a controller with relative degree -1 (PID with negative parameters)
%(s+lambda1)(s+lambda2)(s+lambda3)=*D3s^3+D2s^2+D1s+D0=d3'*[s^3 s^2 s 1];
%(used upper case D so that D3 (constant) and d3 (vector) won't be confused
%The followin matrix is to find the controller paramaters
%Cimp1= tf([q2 q1 q0],[p1 0]);
%Aimp1[q2;q1;q0;p1]=d3;
d3=[1 6 11 6]';
A1imp1=[1 0 0 1;
    -b 1 0 a;
    0 -b 1 0;
    0 0 -b 0];
 param = A1imp1\d3;
 q=[param(1:3)'];
 p=[param(4) 0];
 C1imp1 = tf(q,p);
 Gff1sp1 = 1/C1imp1; 
 %Note on terminology: ff1: feedforward on PLANT 1,
 %                     sp1: STRICTLY PROPER rel. degree 1
 %sp1: strictly proper controller rel. degree 1
 

%%
%%%%%   FF controller with rel. degree 1 for PLANT 2
%%improper2
%To find a controller with relative degree -2
%(s+lambda1)(s+lambda2)(s+lambda3)(s+lambda4)
%           =D4s^4+D3s^3+D2s^2+D1s+d0
%           =d4'*[s^3 s^2 s 1];
%The followin matrix is to find the controller paramaters
%Cimp1= tf([q3 q2 q1 q0],[p1 0]);
%Aimp1[q3;q2;q1;q0;p1]=d4;
d4 = [1 10 35 50 24]';
A1imp2=[1 0 0 0 0;
    -1 1 0 0 1;
    0 -b 1 0 a;
    0 0 -b 1 0;
    0 0 0 -b 0];
param = A1imp2\d4;
q=param(1:4)';
p=[param(5) 0];
C1imp2 = tf(q,p);
Gff1sp2 = 1/C1imp2; %sp2: strictly proper rel. degree 2


%%
%%%%%   FF controller with rel. degree 1 for PLANT 2
%To find a controller with relative degree -1 (PID with negative parameters)
%(s+lambda1)(s+lambda2)(s+lambda3)=*D3s^3+D2s^2+D1s+D0=d3'*[s^3 s^2 s 1];
%(used upper case D so that D3 (constant) and d3 (vector) won't be confused
%The followin matrix is to find the controller paramaters
%Cimp1= tf([q2 q1 q0],[p1 0]);
%Aimp1[q2;q1;q0;p1]=d3;
d3=[1 6 11 6]';
A2imp1=[1 0 0 1;
    -b 1 0 -a;
    0 -b 1 0;
    0 0 -b 0];
 param = A2imp1\d3;
 q=[param(1:3)'];
 p=[param(4) 0];
 C2imp1 = tf(q,p);
 Gff2sp1 = 1/C2imp1; 
 %Note on terminology: ff1: feedforward on PLANT 1,
 %                     sp1: STRICTLY PROPER rel. degree 1
 %sp1: strictly proper controller rel. degree 1
 
 %%
 %%%%%%%%%%%%%%%%%         PLANT 2         %%%%%%%
%%

%%%%%   FF controller with rel. degree 1 for PLANT 2
%%improper2
%To find a controller with relative degree -2
%(s+lambda1)(s+lambda2)(s+lambda3)(s+lambda4)
%           =D4s^4+D3s^3+D2s^2+D1s+d0
%           =d4'*[s^3 s^2 s 1];
%The followin matrix is to find the controller paramaters
%Cimp1= tf([q3 q2 q1 q0],[p1 0]);
%Aimp1[q3;q2;q1;q0;p1]=d4;
d4 = [1 10 35 50 24]';
A2imp2=[1 0 0 0 0;
    -1 1 0 0 1;
    0 -b 1 0 -a;
    0 0 -b 1 0;
    0 0 0 -b 0];
param = A2imp2\d4;
q=param(1:4)';
p=[param(5) 0];
C2imp2 = tf(q,p);
Gff2sp2 = 1/C2imp2; %sp2: strictly proper rel. degree 2


%% PLANT 3
%place poles a -1,-2,-3,-4
d5 = [1 15 85 225 274 120]';
alpha = 2*zeta*w_n;
A3imp2 = [1 zeros(1,5);
          alpha 1 zeros(1,4);
          w_n^2 alpha 1 0 0 0;
          0 w_n^2 alpha 1 1 0;
          0 0 w_n^2 alpha -b 1;
          0 0 0 w_n^2 0 -b];
  param = A3imp2\d5;
  q = param(1:4)';
  p = param(4:6)';
  C3imp2 = tf(q,p);
  Gff3sp2 = 1/C3imp2; 
          
