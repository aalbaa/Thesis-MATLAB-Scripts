a = 1;
b = 2;

w_n = 10;
zeta = -0.5;

Gp1 = tf([1 -b],[1 a 0]);

Gp2 = tf([1 -b],[1 -a 0]);

Gp3 = tf([1 -b],[1 2*w_n*zeta w_n^2]);

Gp4 = tf([1 -b],[1 10 100]); %like Gp3 but stable

