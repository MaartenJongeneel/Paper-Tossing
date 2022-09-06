function [P,et,mu] = Pnc_slipVfit(Bv_TBa,Bv_TBe,Wi,Wa,We,M,dt)
% Using linear least squares to fit Newton-Coulomb parameters assuming slip


% Generalized forces 
f = [Wi(1:3,1:3), zeros(3); zeros(3), Wi(1:3,1:3)]*[0,0,-M(1,1)*9.81,0,0,0]';

% Use only the linear influence of W 
Win = Wi(:,3); 
Wit = Wi(:,1:2);
Wat = Wa(:,1:2);
Wet = We(:,1:2);

% Double cross product v
vdc = [VecToso3(Bv_TBa(4:6)), zeros(3); 
       VecToso3(Bv_TBa(1:3)), VecToso3(Bv_TBa(4:6))]; 

% Set up equations
A = M\[Wit*(Wet'*Bv_TBe), Wit*(Wat'*Bv_TBa), Win];
b = (Bv_TBe - Bv_TBa) - M\(f*dt - vdc*(M*Bv_TBa)*dt);

% disp(vecnorm([Wit*(Wet'*Bv_TBe), Wit*(Wat'*Bv_TBa), Win]))
sig = A\b;

et = sig(2)/sig(1);
P  = sig(1)*(Wet'*Bv_TBe + et*Wat'*Bv_TBa);
P(3) = sig(3);
mu = norm(P(1:2))/P(3);
end