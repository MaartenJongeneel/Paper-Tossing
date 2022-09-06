function [Bv_TBe,P,mu] = Pnc_stick(Bv_TBa,Wi,Wa,M,et,en,dt)
% Using linear least squares to fit Newton-Coulomb parameters assuming stick

% Use only the linear influence of W 
Wi = Wi(:,1:3);
Wa = Wa(:,1:3);

% Generalized forces 
f = [Wi(1:3,1:3), zeros(3); zeros(3), Wi(1:3,1:3)]*[0,0,-M(1,1)*9.81,0,0,0]';

% Determine pre- and post impact point velocity
Tdo_Pa = Wa'*Bv_TBa;
Tdo_Pe = -[et;et;en].*Tdo_Pa;

% Double cross product v
vdc = [VecToso3(Bv_TBa(4:6)), zeros(3); 
       VecToso3(Bv_TBa(1:3)), VecToso3(Bv_TBa(4:6))]; 
   
% Set up equations
A = [eye(6), -M\Wi; Wi', zeros(3,3)];
b = [Bv_TBa + M\(f*dt - vdc*(M*Bv_TBa)*dt); Tdo_Pe];

% Solve linear equation
x = A\b;

Bv_TBe = x(1:6);
P = x(7:9);
end
