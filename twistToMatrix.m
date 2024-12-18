function xi_hat = twistToMatrix(xi)
    % twistToMatrix: Converts a twist vector xi into its corresponding 4x4 matrix representation
    %
    % Input:
    %   xi: 6x1 twist vector [v; omega]
    %
    % Output:
    %   xi_hat: 4x4 matrix representation of the twist (element of se(3))

    % Validate the input
    if length(xi) ~=6
        error('xi must be a 6x1 vector');
    end

    v = xi(1:3);
    omega = xi(4:6);

    omega_hat = [  0       -omega(3)  omega(2);
                omega(3)     0       -omega(1);
               -omega(2)  omega(1)      0    ];

    xi_hat = [omega_hat, v; 0, 0, 0, 0];
end
