%% week 9: November 27
%comparing bi-proper controllers (static gain, 1st and 2nd degree) and
%strictly proper controllers (rel degree 1 and 2) to investigate whether
%the structure of the controller limits us in any way (how big is the
%offset between G and Gbar?).
%we use the same transfer functions in TFs.m

%Terminology: Cijk: controller for plant i with denomenator of degree j and numerator of degree k.
% %of relative degree j (-1 will still be written as 1), and the degree of the
% %numerator is k.
%So C101 is plant 1 and looks something like (q1*s+q0)/(p0).
%Similar terminology is used for the feedforward controllers Gff.

TFs;

%% Plant 1
%%%%%%%%%%%%%%%%%%%%%%%%%   BI-PROPER     %%%%%%%%%%%%%%%%%%%%%
%%%  STATIC GAIN: not possible (check rlocus)
K100 = maxGain(Gp1);
C100 = K100;
Gff100 = 1/C100;
Gbar100 = Gp1 + Gff100;


%%% bi-proper, degree of numerator/denomenator = 1
C111 = polePlacement(Gp1,1,1);
K111 = maxGain(Gp1*C111);
C111 = C111*K111;
Gff111 = 1/C111;
Gbar111 = Gp1 + Gff111;

%%% bi-proper, degree of numerator/denomenator = 2
C122 = polePlacement(Gp1,2,2);
K122 = maxGain(Gp1*C122);
C122 = C122*K122;
Gff122 = 1/C122;
Gbar122 = Gp1 + Gff122;



%%% bi-proper, degree of numerator/denomenator = 2
C122_2 = polePlacement(Gp1,2,2,[-100,-200,-300,-400]);
K122_2 = maxGain(Gp1*C122_2);
C122_2 = C122_2*K122_2;
Gff122_2 = 1/C122_2;
Gbar122_2 = Gp1 + Gff122_2;



%%% bi-proper, degree of numerator/denomenator = 2
C122_3 = polePlacement(Gp1,2,2,[-0.1,-0.2,-0.3,-0.4]);
K122_3 = maxGain(Gp1*C122_3);
C122_3 = C122_3*K122_3;
Gff122_3 = 1/C122_3;
Gbar122_3 = Gp1 + Gff122_3;




%%%%%%%%%%%%%%%%%%%%%%%%%   IMPROPER CONTROLLERS     %%%%%%%%%%%%%%%%%%%%%
%rel. degree = -1!!!!!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q1*s+q0)/p0
C101 = polePlacement(Gp1,0,1);
K101 = maxGain(Gp1*C101);
C101 = C101*K101;
Gff101 = 1/C101;
Gbar101 = Gp1 + Gff101;



%rel. degree = -2 !!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q2*s^2+q1*s+q0)/p0
C102 = polePlacement(Gp1,0,2);
K102 = maxGain(Gp1*C102);
C102 = C102*K102;
Gff102 = 1/C102;
Gbar102 = Gp1 + Gff102;

%rel. degree = -2 !!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q3*s^3+q2*s^2+q1*s+q0)/(p1*s+p0)
C113 = polePlacement(Gp1,1,3);
K113 = maxGain(Gp1*C113);
C113 = C113*K113;
Gff113 = 1/C113;
Gbar113 = Gp1 + Gff113;






%% Plant 2
%%%%%%%%%%%%%%%%%%%%%%%%%   BI-PROPER     %%%%%%%%%%%%%%%%%%%%%
%%%  STATIC GAIN
K200 = maxGain(Gp2);
C200 = K200;
Gff200 = 1/C200;
Gbar200 = Gp2 + Gff200;

%%% bi-proper, degree of numerator/denomenator = 1
C211 = polePlacement(Gp2,1,1);
K211 = maxGain(Gp2*C211);
C211 = C211*K211;
Gff211 = 1/C211;
Gbar211 = Gp2 + Gff211;

%%% bi-proper, degree of numerator/denomenator = 2
C222 = polePlacement(Gp2,2,2);
K222 = maxGain(Gp2*C222);
C222 = C222*K222;
Gff222 = 1/C222;
Gbar222 = Gp2 + Gff222;




%%%%%%%%%%%%%%%%%%%%%%%%%   IMPROPER CONTROLLERS     %%%%%%%%%%%%%%%%%%%%%
%rel. degree = -1!!!!!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q1*s+q0)/p0
C201 = polePlacement(Gp2,0,1);
K201 = maxGain(Gp2*C201);
C201 = C201*K201;
Gff201 = 1/C201;
Gbar201 = Gp2 +  Gff201;



%rel. degree = -2 !!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q2*s^2+q1*s+q0)/p0
C202 = polePlacement(Gp2,0,2);
K202 = maxGain(Gp2*C202);
C202 = C202*K202;
Gff202 = 1/C202;
Gbar202 = Gp2 + Gff202;

%rel. degree = -2 !!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q3*s^3+q2*s^2+q1*s+q0)/(p1*s+p0)
C213 = polePlacement(Gp2,2,4);
K213 = maxGain(Gp2*C213);
C213 = C213*K213;
Gff213 = 1/C213;
Gbar213 = Gp2 + Gff213;



%% Plant 3
%%%%%%%%%%%%%%%%%%%%%%%%%   BI-PROPER     %%%%%%%%%%%%%%%%%%%%%
%%%  STATIC GAIN
K300 = maxGain(Gp3);
C300 = K300;
Gff300 = 1/C300;
Gbar300 = Gp3 + Gff300;

%%% bi-proper, degree of numerator/denomenator = 1
C311 = polePlacement(Gp3,1,1);
K311 = maxGain(Gp3*C311);
C311 = C311*K311;
Gff311 = 1/C311;
Gbar311 = Gp3 + Gff311;

%%% bi-proper, degree of numerator/denomenator = 2
[u x y]= polePlacement(Gp3,2,2);
C322=u;
K322 = maxGain(Gp3*C322);
C322 = C322*K322;
Gff322 = 1/C322;
Gbar322 = Gp3 + Gff322;




%%%%%%%%%%%%%%%%%%%%%%%%%   IMPROPER CONTROLLERS     %%%%%%%%%%%%%%%%%%%%%
%rel. degree = -1!!!!!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q1*s+q0)/p0
C301 = polePlacement(Gp3,0,1);
K301 = maxGain(Gp3*C301);
C301 = C301*K301;
Gff301 = 1/C301;
Gbar301 = Gp3 + Gff301;



%rel. degree = -2 !!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q2*s^2+q1*s+q0)/p0
C302 = polePlacement(Gp3,0,2);
K302 = maxGain(Gp3*C302);
C302 = C302*K302;
Gff302 = 1/C302;
Gbar302 = Gp3 + Gff302;

%rel. degree = -2 !!!!!!!!!!!!!!!!!!!!!!!
%so controller somthing like (q3*s^3+q2*s^2+q1*s+q0)/(p1*s+p0)
C313 = polePlacement(Gp3,1,3);
K313 = maxGain(Gp3*C313);
C313 = C313*K313;
Gff313 = 1/C313;
Gbar313 = Gp3 + Gff313;
