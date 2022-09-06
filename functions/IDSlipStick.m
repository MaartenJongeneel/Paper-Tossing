% Function to identify which contacts are sticking and which are slipping
function [stick, s] = IDSlipStick(P3,Pv_TBa,Pv_TBe,et,thres)
if nargin < 5,    thres = 1e-5; end
if nargin < 4,    et    = 0.35; end

if iscell(P3), P3 = cell2mat(P3'); end

% If the contact sticks, Ft < mu*Fn for stick, so r = Ft/Fn should be smaller than mu
m = norm(P3(1:2))/P3(3);

% If the contact sticks, the contact point velocity reverses
r = 1+(Pv_TBe(1:2)/norm(Pv_TBe(1:2)))' * (Pv_TBa(1:2)/norm(Pv_TBa(1:2)));
 
% If the contact sticks, xi should be small (even zero) for stick
xi = Pv_TBe(1:2) + et*Pv_TBa(1:2);
% disp('---');
% disp(norm(xi))

% If the contact SLIPS, xi is in opposite direction of P
a = 1-(P3(1:2)/norm(P3(1:2)))' * (xi(1:2)/norm(xi(1:2)));

% Combine the two criteria -> lower = higher chance it's stick
s = m^2 * r^2 * norm(xi)^2 * a^2;

% Decide based on threshold if stick
stick = s<thres;
end