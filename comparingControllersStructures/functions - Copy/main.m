clear all;
close all;

TFs;

[Gbar C Gfb sol] = polePlacement(Gp3,1,1);

RH = rhStabilityCriterion(conv(sol(1:2),Gp3.denominator{1,1}));