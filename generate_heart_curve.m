function heart_points = generate_heart_curve(center, scale, num_points, start_point)
% generate_heart_curve: Generates a dense 3D heart shape curve with a specific start point
%
% Inputs:
%   center - 1x3 vector specifying the initial center of the heart [x, y, z]
%   scale - scalar value to control the size of the heart (default = 1)
%   num_points - number of points to generate for the heart curve (default = 1000)
%   start_point - 1x2 vector specifying the desired start point [x_start, y_start]
%
% Output:
%   heart_points - N×3 array where each row is a point [x, y, z] on the curve
%
% Example usage:
%   heart_points = generate_heart_curve([0.3, 0.5, 0.22], 0.05, 1000, [0.25, 0.60]);
%   plot3(heart_points(:,1), heart_points(:,2), heart_points(:,3), 'm');

    % Check input arguments
    if nargin < 4
        start_point = []; % No specific start point
    end
    if nargin < 3
        num_points = 1000; % Default number of points
    end
    if nargin < 2
        scale = 1; % Default scale
    end
    if nargin < 1
        center = [0, 0, 0]; % Default center
    end

    % Generate the theta values
    theta_vals = linspace(0, 2*pi, num_points);
    heart_points = zeros(num_points, 3);

    % Loop to generate the heart points
    for i = 1:num_points
        x = scale * 16 * sin(theta_vals(i))^3;
        y = scale * (13 * cos(theta_vals(i)) - 5 * cos(2*theta_vals(i)) - 2 * cos(3*theta_vals(i)) - cos(4*theta_vals(i)));
        z = 0; % Keep Z constant
        heart_points(i,:) = [x, y, z];
    end

    % Adjust for specific start point if provided
    if ~isempty(start_point)
        % Compute current start point
        x_offset = start_point(1) - (heart_points(1,1) + center(1));
        y_offset = start_point(2) - (heart_points(1,2) + center(2));

        % Update center to align with start point
        center(1) = center(1) + x_offset;
        center(2) = center(2) + y_offset;
    end

    % Apply center adjustment to the curve
    heart_points(:,1) = heart_points(:,1) + center(1);
    heart_points(:,2) = heart_points(:,2) + center(2);
    heart_points(:,3) = heart_points(:,3) + center(3);
end
