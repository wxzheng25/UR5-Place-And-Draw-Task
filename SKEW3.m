function R = SKEW3(x)

% SKEW3(): accepts a 3 × 1 vector x = [x1, x2, x3]⊤ 
% and returns the corresponding canonical 3 × 3 skew-symmetric matrix
%
% Input:
%    x: a 3 × 1 vector x = [x1, x2, x3]⊤
%
% Output:
%    R: 3 × 3 skew-symmetric matrix
% Aurthor: Wenxuan Zheng, Chang Liu, Chenhao Yu, Xingyu Wang
% Date:    20241115

R = [  0   -x(3)  x(2);
      x(3)   0    -x(1);
     -x(2)  x(1)     0];

end
