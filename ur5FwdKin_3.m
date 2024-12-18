function gBaseTool = ur5FwdKin_3(q)
    % ur5FwdKin: Computes the forward kinematics of the UR5 robot using the DH convention.
    %
    % Inputs:
    %   q: A 6x1 vector of joint angles (in radians) for the UR5, typically in range [-pi, pi].
    %
    % Output:
    %   gBaseTool: A 4x4 homogeneous transformation matrix representing the pose of
    %              the tool0 frame relative to the base_link frame.
    %
    % Note: This function is designed to be consistent with the provided ur5InvKin function.
    %       The ur5InvKin code applies a correction of "-pi" to the first joint angle.
    %       To maintain consistency, we add "+pi" to q(1) here.

    % DH parameters (same as used in ur5InvKin)
    d1 = 0.089159;
    d4 = 0.10915;
    d5 = 0.09465;
    d6 = 0.0823;
    a2 = -0.425;
    a3 = -0.39225;

    alpha1 = pi/2;
    alpha2 = 0;
    alpha3 = 0;
    alpha4 = pi/2;
    alpha5 = -pi/2;
    alpha6 = 0;

    a1 = 0; a4 = 0; a5 = 0; a6 = 0; d2 = 0; d3 = 0;

    % Adjust q(1) by +pi to match the inverse kinematics convention
    q(1) = q(1) + pi;

    % Compute transformations for each joint using the DH function
    T01 = DH(a1, alpha1, d1, q(1));
    T12 = DH(a2, alpha2, d2, q(2));
    T23 = DH(a3, alpha3, d3, q(3));
    T34 = DH(a4, alpha4, d4, q(4));
    T45 = DH(a5, alpha5, d5, q(5));
    T56 = DH(a6, alpha6, d6, q(6));

    % Multiply all transformations to get the final pose
    gBaseTool = T01 * T12 * T23*T34 ;
end


