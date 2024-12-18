function xi = twistify(X)
    % twistify: Converts a 4x4 matrix to a 6x1 twist vector.
    %
    % Inputs:
    %   X: 4x4 matrix, typically representing an element of se(3)
    %
    % Outputs:
    %   xi: 6x1 twist vector [v; omega]
    %
    % Extract the rotational part (omega_hat) and the translational part (v)
    % Aurthor: Wenxuan Zheng, Chang Liu, Chenhao Yu, Xingyu Wang
    % Date:    20241115
    omega_hat = X(1:3, 1:3);
    v = X(1:3, 4);

    % Ensure omega_hat is skew-symmetric
    omega_hat = (omega_hat - omega_hat') / 2;

    % Extract omega from omega_hat
    omega = [omega_hat(3, 2); omega_hat(1, 3); omega_hat(2, 1)];

    % Combine v and omega into the twist vector
    xi = [v; omega];
end
