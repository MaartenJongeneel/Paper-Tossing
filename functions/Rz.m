function R = Rz(th)
%Rotate around z with th degrees;
R = [cos(deg2rad(th)) -sin(deg2rad(th)) 0; sin(deg2rad(th)) cos(deg2rad(th)) 0; 0 0 1];