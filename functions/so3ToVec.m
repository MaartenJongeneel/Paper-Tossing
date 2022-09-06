function omg = so3ToVec(so3mat)
% This function takes a 3x3 skew-symmetric matrix (an element of so(3)).
% Returns the corresponding 3-vector (angular velocity).
% 
% INPUTS:    so3mat       : 3x3 skew-symmetric matrix
%
% OUTPUTS:   omg          : 3-vector (angular velocity)
%
% Example Input: 
% 
% so3mat = [[0, -3, 2]; [3, 0, -1]; [-2, 1, 0]];
% omg = so3ToVec(so3mat)  
% 
% Output:
% omg =
%     1
%     2
%     3
%% so3ToVec
omg = [so3mat(3, 2); so3mat(1, 3); so3mat(2, 1)];
end