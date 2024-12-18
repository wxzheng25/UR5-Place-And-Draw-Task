
function gst = ur5FwdKin3(q)
    % ur5FwdKin: Computes the forward kinematics of the UR5 robot.
    % 
    % Inputs:
    %   q: 6x1 joint space variable vector = [theta1, theta2, theta3, theta4, theta5, theta6]
    %      where theta_n is the joint angle of joint n.
    % 
    % Outputs:
    %   gst: 4x4 homogeneous transformation matrix of the end-effector pose.
    %

[rows, cols] = size(q);
    if ((rows ~= 6) || (cols ~= 1))
        error('ur5FwdKin requires a 6-by-1 joint vector argument. Check your dimensions.');
    end

    % Define the UR5 link lengths and offsets (in meters) 
    L0 = 0.0892;   % Base height
    L1 = 0.425;    % Link 1 length
    L2 = 0.392;    % Link 2 length
    L3 = 0.1093;   % Wrist 1 offset
    L4 = 0.09475;  % Wrist 2 offset
    L5 = 0.0825;   % Wrist 3 offset

    % Define joint axes (w) and points (q) 
    w1 = [0; 0; 1];    q1 = [0; 0; 0];
    w2 = [1; 0; 0];    q2 = [0; 0; 0];
    w3 = w2;            q3 = [0; 0; L1];
    w4 = w2;            q4 = [0; 0; L1 + L2];
    w5 = [0; 0; 1];     q5 = [L3; 0; 0];
    w6 = w2;            q6 = [0; 0; L1 + L2 + L4];

    % Compute v_i = -w_i x q_i
    v1 = -cross(w1, q1);
    v2 = -cross(w2, q2);
    v3 = -cross(w3, q3);
    v4 = -cross(w4, q4);
    v5 = -cross(w5, q5);
    v6 = -cross(w6, q6);

    % Assemble the screw axes ξ = [v; w]
    S1 = [v1; w1];
    S2 = [v2; w2];
    S3 = [v3; w3];
    S4 = [v4; w4];
    S5 = [v5; w5];
    S6 = [v6; w6];

    % adjust orientation to match Rviz:
    G_adj = [ROTZ(pi/2) [0;0;0.0892]; 0 0 0 1];
    S1 = Adjoint(G_adj)*S1;
    S2 = Adjoint(G_adj)*S2;
    S3 = Adjoint(G_adj)*S3;
    S4 = Adjoint(G_adj)*S4;
    S5 = Adjoint(G_adj)*S5;
    S6 = Adjoint(G_adj)*S6;

    % Define the home configuration gst0 
    % gst0 = [ROTX(-pi/2) [0; L3+L5; L1+L2+L4+0.0892]; 0 0 0 1]
    R0 = ROTX(-pi/2);
    p0 = [0; L3+L5; L1 + L2 + L4 + 0.0892];
    gst0 = [R0 p0; 0 0 0 1];

    % Compute forward kinematics using POE from the last joint to the first
    gst = gst0;
    gst = expm(twist_hat(S6)*q(6)) * gst;
    gst = expm(twist_hat(S5)*q(5)) * gst;
    gst = expm(twist_hat(S4)*q(4)) * gst;
    gst = expm(twist_hat(S3)*q(3)) * gst;
    gst = expm(twist_hat(S2)*q(2)) * gst;
    gst = expm(twist_hat(S1)*q(1)) * gst;

end

% Functions
function Adg = Adjoint(g)
    R = g(1:3,1:3);
    p = g(1:3,4);
    p_hat = Hat(p);
    Adg = [R p_hat*R; zeros(3) R];
end

function R = ROTX(alpha)
    R = [1 0 0;
         0 cos(alpha) -sin(alpha);
         0 sin(alpha) cos(alpha)];
end

function R = ROTZ(theta)
    R = [cos(theta), -sin(theta), 0;
         sin(theta),  cos(theta), 0;
         0,           0,          1];
end

function R = Rodrigues(w, theta)
    if abs(norm(w)-1)>0.001
        error('w must be a unit vector.');
    end
    w_hat = Hat(w);
    R = eye(3) + w_hat*sin(theta) + (w_hat^2)*(1 - cos(theta));
end

function w_hat = Hat(w)
    w_hat = [0    -w(3)  w(2);
             w(3)  0    -w(1);
            -w(2) w(1)   0   ];
end

function xi_hat = twist_hat(xi)
% Converts a twist vector xi into its corresponding 4x4 matrix representation
    omega = xi(4:6);
    v = xi(1:3);
    omega_hat = Hat(omega);
    xi_hat = [omega_hat v;0 0 0 0];
end
