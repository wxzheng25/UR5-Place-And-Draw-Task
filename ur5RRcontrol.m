function finalerr = ur5RRcontrol(gdesired, K, ur5)
    % ur5RRcontrol: Implements a discrete-time resolved-rate control system for the UR5 robot.
    %
    % Inputs:
    %   gdesired: 4x4 desired end-effector pose (homogeneous transformation matrix)
    %   K: Gain of the controller
    %   ur5: ur5_interface object
    %
    % Output:
    %   finalerr: -1 if failure (e.g., singularity encountered), else the final positional error in cm
    %
    % Aurthor: Wenxuan Zheng, Chang Liu, Chenhao Yu, Xingyu Wang
    % Date:    20241115

    % Initialize parameters
    threshold_v = 0.003; 
    threshold_omega = deg2rad(1); 
    T_step = 0.03; % Time step in seconds
    epsilon = 1e-3; % Threshold for manipulability measure to detect singularity

    % Get initial joint configuration
    q_k = ur5.get_current_joints(); 
    q_k2=q_k;
    q_k(2)=q_k(2)+pi/2;
    q_k(4)=q_k(4)+pi/2;
    max_iterations = 1000; % Maximum number of iterations
    iteration = 0;

    while iteration < max_iterations
        iteration = iteration + 1;

        % Compute current end-effector pose
        g_st = ur5FwdKin3(q_k);

        % Compute error transformation
        g_error = inv(gdesired) * g_st;

        % Compute error twist xi_k
        xi_k = getXi(g_error);

        % Compute norms of translational and rotational components
        norm_v = norm(xi_k(1:3));
        norm_omega = norm(xi_k(4:6));
        disp(norm_v);
        disp(norm_omega);
        % Check termination condition
        if norm_v < threshold_v && norm_omega < threshold_omega
            finalerr = norm_v * 100; % Convert to cm
            disp('Converged to the desired pose.');
            return;
        end

        % Compute body Jacobian at current configuration
        J_stb = ur5BodyJacobian3(q_k);

        % Check for singularity using minimum singular value
        sigma_min = min(svd(J_stb));
        if sigma_min < epsilon
            % Near singularity, abort and return -1
            disp('Encountered singularity.');
            finalerr = -1;
            return;
        end

        % Compute joint angle update
        delta_q = -K * T_step * (inv(J_stb) * xi_k); % Using matrix left division for numerical stability

        % Update joint angles
        q_k2 = q_k2 + delta_q;

        % Move robot to new joint configuration
        ur5.move_joints(q_k2, T_step);

        pause(T_step); % Wait
        q_k=q_k2;
        q_k(2)=q_k2(2)+pi/2;
        q_k(4)=q_k2(4)+pi/2;
    end

    disp('Maximum iterations reached without convergence.');
    finalerr = -1;
end
