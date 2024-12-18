% ur5BodyJacobian.m
%
% Purpose: Compute the Jacobian matrix for the UR5. All necessary parameters are to be 
%          defined inside the function. Again, parameters such as twists and gst(0) 
%          (if needed) should be defined in the function.
% q: 6 × 1 joint space variables vector
% Output: J: Body Jacobian, Jsbt
%
% function : FINV(), SKEW3()

function Jst_body = ur5BodyJacobian3(Q)
    % Check input
    [rows, cols] = size(Q);
    if (rows ~= 6 || cols ~= 1)
        error('ur5BodyJacobian: Q should be a 6x1 vector of joint angles.');
    end

    %% UR5 Parameters (Based on ur5BodyJacobian3 logic)
    % Link lengths (in meters)
    L = [425 392.25 109.15 94.65 82.3]*0.001; % Convert mm to m

    % Define joint axes (w) and points (q) in their initial configuration
    w = zeros(3,6);
    w(:,1) = [0; 0; 1];
    w(:,2) = [1; 0; 0];
    w(:,3) = w(:,2);
    w(:,4) = w(:,2);
    w(:,5) = w(:,1);
    w(:,6) = w(:,2);

    q = zeros(3,6);
    q(:,1) = [0; 0; 0];
    q(:,2) = q(:,1);
    q(:,3) = [0; 0; L(1)];
    q(:,4) = [0; 0; L(1)+L(2)];
    q(:,5) = [L(3); 0; 0];
    q(:,6) = [0; 0; L(1)+L(2)+L(4)];

    %% Compute Twists ξ_i for each joint
    xi = zeros(6,6);
    for i = 1:6
        v = -cross(w(:,i), q(:,i));
        xi(:,i) = [v; w(:,i)];

        Rz = [cos(pi/2) -sin(pi/2) 0;
              sin(pi/2)  cos(pi/2) 0;
              0          0         1]; % ROTZ(pi/2)
        p_adj = [0;0;0.0892];
        p_hat = Hat(p_adj);

        Adg = [Rz p_hat*Rz; zeros(3) Rz];
        xi(:,i) = Adg*xi(:,i);
    end

    %% Define gst0 using logic from ur5BodyJacobian3
    % ROTX(-pi/2)
    Rx = [1 0          0;
          0 cos(-pi/2) -sin(-pi/2);
          0 sin(-pi/2) cos(-pi/2)];

    p0 = [0; L(3)+L(5); (L(1)+L(2)+L(4)+0.0892)];
    gst0 = [Rx p0; 0 0 0 1];

    %% Compute current gst from Q
    gst = gst0;
    for i = 6:-1:1
        xi_i = xi(:,i);
        theta = Q(i);
        v = xi_i(1:3);
        w_i = xi_i(4:6);
        if norm(w_i)<1e-9
            % Pure translation
            g_i = [eye(3) theta*v;0 0 0 1];
        else
            w_unit = w_i/norm(w_i);
            % Rodrigues formula
            w_hat = Hat(w_unit);
            R = eye(3) + w_hat*sin(theta) + (w_hat^2)*(1 - cos(theta));
            g_i = [R (eye(3)-R)*cross(w_unit,v)+(w_unit*w_unit')*v*theta;0 0 0 1];
        end
        gst = g_i * gst;
    end

    %% Spatial Jacobian Js
    Js = zeros(6,6);
    Js(:,1) = xi(:,1);
    for i = 2:6
        g_intermediate = eye(4);
        for j = 1:i-1
            xi_j = xi(:,j);
            theta_j = Q(j);
            v_j = xi_j(1:3);
            w_j = xi_j(4:6);
            if norm(w_j)<1e-9
                % Pure translation
                g_j = [eye(3) theta_j*v_j;0 0 0 1];
            else
                w_uj = w_j/norm(w_j);
                w_hatj = Hat(w_uj);
                Rj = eye(3) + w_hatj*sin(theta_j) + (w_hatj^2)*(1 - cos(theta_j));
                g_j = [Rj (eye(3)-Rj)*cross(w_uj,v_j)+(w_uj*w_uj')*v_j*theta_j;0 0 0 1];
            end
            g_intermediate = g_intermediate * g_j;
        end
        % Adjoint of g_intermediate:
        R_temp = g_intermediate(1:3,1:3);
        p_temp = g_intermediate(1:3,4);
        p_hat_temp = Hat(p_temp);
        Ad_g_inter = [R_temp p_hat_temp*R_temp; zeros(3) R_temp];
        Js(:,i) = Ad_g_inter*xi(:,i);
    end

    %% Body Jacobian Calculation

    Rf = gst(1:3,1:3);
    pf = gst(1:3,4);
    Rf_t = Rf';
    gst_inv = eye(4);
    gst_inv(1:3,1:3) = Rf_t;
    gst_inv(1:3,4) = -Rf_t*pf;

    % Adjoint(gst_inv)*Js
    R_inv = gst_inv(1:3,1:3);
    p_inv = gst_inv(1:3,4);
    p_hat_inv = Hat(p_inv);
    Ad_gst_inv = [R_inv p_hat_inv*R_inv; zeros(3) R_inv];

    Jst_body = Ad_gst_inv*Js;

end

function w_hat = Hat(w)
%Hat(w): Convert a 3x1 vector into skew-symmetric matrix
    if ~isequal(size(w), [3,1])
        error('Hat requires a 3x1 vector.');
    end
    w_hat = [0 -w(3) w(2);
             w(3) 0 -w(1);
             -w(2) w(1) 0];
end
