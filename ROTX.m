function R = ROTX(theta)

% ROTX: 3x3 rotation matrix about the X-axis 
%
% Input:
%    theta: a scalar roll value (in radians)
%
% Output:
%    R: 3 × 3 rotation matrix

R = [1      0               0;
     0 cos(theta) -sin(theta);
     0 sin(theta)  cos(theta)];

end