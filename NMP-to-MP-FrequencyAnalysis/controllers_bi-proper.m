% controller: G0i, where i is the plant's number
%using root locus to check whether we can use gain only to construct the
%Gff controller (=1/maxGain)

%% Gain-only controllers
%%
k0 = maxGain(Gp1);
%remember that a valid K0 is a k that is positive
if k0>0
    G01 = k0;
    Gff1 = 1/k0;
    Gbar1 = Gp1 + Gff1;
else
    G01 = 0; %no answer basically
end
%%
k0 = maxGain(Gp2);
if k0>0
    G02 = k0;
    Gff2 = 1/k0;
    Gbar2 = Gp2 + Gff2;
else
    G02 = 0;
end
%%
k0 = maxGain(Gp3);
if k0>0
    G03 = k0;
    Gff3 = 1/k0;
    Gbar3 = Gp3 + Gff3;
else
    G03 = 0;    
end



%% higher order b-proper controllers
% polePlacement() function is used here.