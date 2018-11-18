% This function returns the Descriptor State Space matrices (A,B,C,D,E) 
% such that D is nonzero
%for SISO systems!
function [Ads, Bds, Cds, Dds, Eds] = ss2dssD(G)

Ap = G.A;
Bp = G.B;
Cp = G.C;
Dp = G.D;
Ep = G.E;

n = size(Ap,1);

if size(Ep,1)==0
    Ep = eye(n);
end



if norm(Dp)<0.01        
        Eds = Ep; Eds(n+1,n+1)=0;

        Ads = [Ap, zeros(n,1);
                zeros(1,n), 1];

        Bds = [Bp;1];

        Cds = [Cp, 1];

        Dds = 1;
else
    Eds = Ep;
    Ads = Ap;
    Bds = Bp;
    Cds = Cp;
    Dds = Dp;    
    
%     
% if rank(Eds)==size(Eds,1)
%     Eds = blkdiag(Eds,0);
%     Ads = blkdiag(Ads,1);
%     Bds = [Bds; 1];
%     Cds = [Cds, 1];
%     Dds = [Dds+1];
% end

end



    