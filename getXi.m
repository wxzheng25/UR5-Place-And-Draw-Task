function xi = getXi(g)
    % getXi: Extracts the unnormalized twist vector xi from a homogeneous transformation matrix g
    %
    % Input:
    %   g: 4x4 homogeneous transformation matrix
    %
    % Output:
    %   xi: 6x1 twist vector such that g = expm(hat(xi))
    %
    % Note:
    %   This function computes the twist vector xi in se(3) corresponding to the transformation g.
    %   It does not normalize xi.


    % Validate the input
    if ~isequal(size(g), [4, 4]) || ~isequal(g(4, :), [0, 0, 0, 1])
        error('Input must be a valid 4x4 homogeneous transformation matrix.');
    end

    % Extract rotation matrix R and translation vector p
    R = g(1:3, 1:3);
    p = g(1:3, 4);

    % Compute the angle theta from the rotation matrix
    theta = acos((trace(R) - 1) / 2);

    % Handle the case when theta is very small (approximate to zero)
    if abs(theta) < 1e-6
        % Pure translation
        omega = zeros(3, 1);
        v = p;
    else
        % Compute the skew-symmetric matrix of omega
        omega_hat = (theta / (2 * sin(theta))) * (R - R');
        omega = [omega_hat(3, 2); omega_hat(1, 3); omega_hat(2, 1)];

        % Compute the matrix A inverse
        A_inv = eye(3) - 0.5 * omega_hat + (1 / theta^2) * (1 - (theta * sin(theta)) / (2 * (1 - cos(theta)))) * (omega_hat^2);

        % Compute v
        v = A_inv * p;
    end

    % Assemble the twist vector xi = [v; omega]
    xi = [v; omega];
end
