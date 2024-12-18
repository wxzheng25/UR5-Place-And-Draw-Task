function R = ROTY(theta)

% ROTX: 3x3 rotation matrix about the Y-axis 
%
% Input:
%    theta: a scalar roll value (in radians)
%
% Output:
%    R: 3 × 3 rotation matrix

R = [cos(theta)  0   sin(theta);
     0           1           0;
     -sin(theta) 0   cos(theta)];

end