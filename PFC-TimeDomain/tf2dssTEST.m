%testing tf2dss function

format long;

a = 1;
b = 2;

w_n = 10;
zeta = -0.5;

num1 = [1 -b];
den1 = [1 a 0];
Gp1 = tf(num1,den1);

num2 = [1 -b];
den2 = [1 -a 0];
Gp2 = tf(num2,den2);

num3 = [1 -b];
den3 = [1 2*w_n*zeta w_n^2];
Gp3 = tf(num3,den3);

[A,B,C,D,E] = tf2dss(num1,den1);
Gp1Test = tf(dss(A,B,C,D,E));
fprintf("norm (Gp1-Gp1TEST) = %u\n",norm(Gp1-Gp1Test));

[A,B,C,D,E] = tf2dss(num2,den2);
Gp2Test = tf(dss(A,B,C,D,E));
fprintf("norm (Gp2-Gp2TEST) = %u\n",norm(Gp2-Gp2Test));

[A,B,C,D,E] = tf2dss(num3,den3);
Gp3Test = tf(dss(A,B,C,D,E));
fprintf("norm (Gp3-Gp3TEST) = %u\n",norm(Gp3-Gp3Test));


%RESULT: SUCCESS