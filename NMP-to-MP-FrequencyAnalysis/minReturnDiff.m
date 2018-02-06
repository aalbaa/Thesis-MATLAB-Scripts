%this function returns the minimum return difference 
%i.e. closes point on the Nyquist plot to (-1,0)
%G is what would be entered to the nyquist() funciton

function [minRD location] = minReturnDiff(G)
[re im] = nyquist(G);
distance =((re+1)+im*i);
mag = abs(distance);
[minRD index]=min(mag(1,1,:));
location = distance(1,1,index)-1;
nyquist(G);
hold on;
plot([real(location) -1],[imag(location) 0],'-xr');
hold off;

