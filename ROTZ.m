function R = ROTZ(theta)

% ROTX: 3x3 rotation matrix about the Z-axis 
%
% Input:
%    theta: a scalar roll value (in radians)
%
% Output:
%    R: 3 × 3 rotation matrix

R = [cos(theta)  -sin(theta)  0;
     sin(theta)   cos(theta)  0;
     0            0           1];


end