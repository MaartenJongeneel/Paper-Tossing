function [P, err] = impulseProj(M,dv,H,p,T)
% Project impulse on the image of either the full or partial Jacobian (3x6
% or 6x6), to determine the impulsive wrench and the error in the impulse.

R = H(1:3,1:3);         % Accepting either rotm or tform

% Contact Jacobian per contact point
W(:,1:3) = transpose([ eye(3)*R,  -R*VecToso3(p)]);
if exist('T','var') && ~isempty(T)
    W(:,4:6) = transpose([ zeros(size(T,1),3),   T*R]);
end
MW = M\W;

% Find projection of dv on the image 3D Jacobian subspace
P = inv(MW'*MW)*MW'*dv;
Proj = MW*P;
err = dv - Proj;        % Error in impulse
end