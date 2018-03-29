% This function returns the Descriptor State Space matrices (A,B,C,D,E) 
% such that D is nonzero
%for SISO systems!
%LOG

% March 20
%   D = 0 if you want D==0
%   or D = 1 if you want D != 0

function [Ads, Bds, Cds, Dds, Eds, SYS_D] = ss2dssD(G,D)
if nargin == 1
    D = 1;
end

[Ap, Bp, Cp, Dp, Ep]  = dssdata(G);

n = size(Ap,1);

if size(Ep,1)==0
    Ep = eye(n);
end


if D == 1
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
    end
elseif D == 0
    if norm(Dp)<0.01      
        Eds = Ep;
        Ads = Ap;
        Bds = Bp;
        Cds = Cp;
        Dds = Dp;        
        
    else
        
        Eds = Ep; Eds(n+1,n+1)=0;

        Ads = [Ap, zeros(n,1);
                zeros(1,n), -1];

        Bds = [Bp;1];

        Cds = [Cp, Dp];

        Dds = 0;           
    end
end
    
SYS_D = dss(Ads,Bds,Cds,Dds,Eds);
end


    