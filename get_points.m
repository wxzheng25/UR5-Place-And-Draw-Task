% gst1 = [0 -1  0  0.25; 
%        -1  0  0  0.60; 
%         0  0 -1  0.22; 
%         0  0  0  1];
% gst2 = [0 -1  0  0.40; 
%        -1  0  0  0.45; 
%         0  0 -1  0.22; 
%         0  0  0  1];
% 
% [d,g3,g4] = form_rectangle_gsts(gst1,gst2);
% 
% disp(g3);
% disp(g4);
% disp(d);

% gst5 = [0 -1  0  0.20; 
%        -1  0  0  0.45; 
%         0  0 -1  0.22; 
%         0  0  0  1];
% gst6 = [0 -1  0  0.40; 
%        -1  0  0  0.45; 
%         0  0 -1  0.22; 
%         0  0  0  1];

% gst1 = [0  -1   0  0.25;
%        -1   0   0  0.60;
%         0    0  -1  0.22;
%         0    0   0  1];
% 
% % Define target pose (g_{st4}) for point (4)
% gst4 = [0  -1   0  0.25+0.0707;
%        -1   0   0  0.60-0.0707;
%         0    0  -1  0.22;
%         0    0   0  1];
% [d,g2,g3] = form_rectangle_gsts(gst1,gst4);
% 
% disp(g2);
% disp(g3);
% disp(d);

function [d, gst_3, gst_4] =get_points(gst_1, gst_2)
    % Input:
    % gst_1, gst_2: Two 4x4 transformation matrices (same rotation R, different positions P)
    %
    % Output:
    % d: The spatial distance between the endpoints defined by gst_1 and gst_2
    % gst_3, gst_4: Two new transformation matrices that, together with gst_1 and gst_2,
    %               form a rectangle of length d and width d/2, while preserving the z-axis direction.

    % Extract rotation and translation
    R1 = gst_1(1:3, 1:3);
    P1 = gst_1(1:3, 4);
    R2 = gst_2(1:3, 1:3);
    P2 = gst_2(1:3, 4);

    % Assume R1 and R2 are the same as stated in the problem
    R = R1; 
    % If needed, you can add a check:
    % assert(norm(R1-R2) < 1e-12, 'R1 and R2 are not identical')

    % Compute the distance d between the two points
    d_vec = P2 - P1;
    d = norm(d_vec);

    % Normalize the direction vector
    dir = d_vec / d;

    % Extract the z-axis from the rotation matrix (assuming it's the third column)
    z_axis = R(:,3);

    % Construct a vector perpendicular to 'dir' while preserving the z-axis direction
    % by taking the cross product of z_axis and dir
    perp_dir = cross(z_axis, dir);
    perp_dir = perp_dir / norm(perp_dir);

    % The width of the rectangle is d/2
    width = d/2;

    % Shift P1 and P2 along the perpendicular direction to form the rectangle
    P3 = P1 + width * perp_dir;
    P4 = P2 + width * perp_dir;

    % Construct gst_3 and gst_4
    gst_3 = [R, P3; 0 0 0 1];
    gst_4 = [R, P4; 0 0 0 1];
end