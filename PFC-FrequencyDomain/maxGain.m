%plots rlocus(G) and
%finds max gain in root locus such that all roots are in the Open Left Half
%Plane (OLHP)
%G is what you would feed the root locus

%Amro Al Baali
%November 11 2017
function [maxK atRoots]= maxGain(G)
% rlocus(G);
[r k] = rlocus(G);



for i=length(r):-1:1
    %if all roots are non-positive
    % basically I convert the logical to a numeric answer and see if all
    % answers are 0 (by taking the norm)(i.e. it is NOT (>= 0) <=> they're all <0)
    if norm(double(r(:,i)>=0))==0
        maxK = k(i);
        atRoots= r(:,i);
        break;
    end
     
    if i == 1
        disp('No Solution');

        maxK = -Inf;
        atRoots = NaN;
    end
end

