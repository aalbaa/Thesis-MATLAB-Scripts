function objFunc = objFun(x,degP,degQ)

Gff = tf(x(degQ+2:degQ+2+degP)',x(1:degQ+1)');

objFunc = norm(Gff);

