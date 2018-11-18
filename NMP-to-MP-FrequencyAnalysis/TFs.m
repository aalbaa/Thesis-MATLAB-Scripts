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

s = tf('s');

