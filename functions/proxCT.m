function y=proxCT(x,a)
% Proximal point formulation for CT. See thesis for reference.
%
% prox_CT(x) = x           for ||x|| =< a
%            = a*(x/||x||) for ||x||  > a
% for CT = {x in R^n| ||x|| =< a}
%
% INPUTS:    x  : Vector in R^n
%
% OUTPUTS:   y  : Vector in R^n 
%% Script
cnt = 1;
for ii = 1:length(a) %For each point in contact
    if norm(x(cnt:cnt+1)) <= a(ii)
        y(cnt:cnt+1,1) = x(cnt:cnt+1); %Stick
    else
        y(cnt:cnt+1,1) = a(ii)*x(cnt:cnt+1)/norm(x(cnt:cnt+1)); %Slip
    end
    cnt = cnt+2;
end


end