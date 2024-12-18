function mu = manipulability(J, measure)
    % manipulability: Computes a measure of manipulability for a given Jacobian.
    %
    % Inputs:
    %   J: 6x6 Jacobian matrix
    %   measure: A string specifying the type of manipulability measure:
    %            - 'sigmamin': Minimum singular value of J
    %            - 'detjac': Square root of determinant of J * J^T
    %            - 'invcond': Inverse of the condition number of J
    %
    % Output:
    %   mu: The computed manipulability measure
    %

    %
    % Validate the inputs
    if ~ismatrix(J) || size(J, 1) ~= 6 || size(J, 2) ~= 6
        error('J must be a 6x6 matrix.');
    end
    if ~ischar(measure) || ~ismember(measure, {'sigmamin', 'detjac', 'invcond'})
        error('measure must be one of "sigmamin", "detjac", or "invcond".');
    end

    % Compute singular values of J
    singular_values = svd(J);

    switch measure
        case 'sigmamin'
            % Minimum singular value of J
            mu = min(singular_values);
            
        case 'detjac'
            % Square root of determinant of J * J^T
            % Equivalent to product of singular values
            mu = prod(singular_values);
            
        case 'invcond'
            % Inverse of the condition number of J
            % Condition number is the ratio of max to min singular values
            mu = singular_values(end) / singular_values(1);
        otherwise
            error('Invalid measure specified.');
    end
end
