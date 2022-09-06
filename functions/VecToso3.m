function so3mat = VecToso3(omg)
% This function takes a 3-vector (angular velocity)..
% Returns the skew symmetric matrix in so(3).
% 
% INPUTS:    omg          : 3-vector (angular velocity)
%
% OUTPUTS:   so3mat       : 3x3 skew-symmetric matrix
%
% Example Input: 
% 
% omg = [1; 2; 3];
% so3mat = VecToso3(omg)
% 
% Output:
% so3mat =
%     0    -3     2
%     3     0    -1
%    -2     1     0
%% so3ToVec
so3mat = [0, -omg(3), omg(2); omg(3), 0, -omg(1); -omg(2), omg(1), 0];
end