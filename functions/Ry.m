function R = Ry(th)
%Rotate around y with th degrees;
R = [cos(deg2rad(th)) 0 sin(deg2rad(th)); 0 1 0;  -sin(deg2rad(th)) 0 cos(deg2rad(th))];