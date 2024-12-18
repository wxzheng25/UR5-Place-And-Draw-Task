function finalerr = JTcontrol(g_target, q_target, K)

T_step = 0.1;
q = ur5.get_current_joints();

gst_current = ur5FwdKin(q);
xi = getXi(g_target \ gst_current);
v = xi(1:3);
w = xi(4:6);
n = 1;

while norm(v) > 0.01 || norm(w) > 1 / 180 * pi 

    % Check the manipulibility
    J = ur5BodyJacobian(q);
    if abs(manipulability(J,'sigmamin')) <0.001
        finalerr = -1;
        return
    end

    diff = q_target - q;
    angle_velo = K * transpose(J) * diff;

    q_new = q + angle_velo * T_step;
    ur5.move_joints(q_new,T_step);
    pause(T_step);

    q = ur5.get_current_joints;
    gst_current = ur5FwdKin(q);
    xi = getXi(g_target \ gst_current);
    v = xi(1:3);
    w = xi(4:6);
    
    if norm(v) <= 0.01 && norm(w) <=1 / 180 * pi
        finalerr = norm_v;
        disp('Converaged to desired pose')
    end

    if n > 1000
        disp('The gain K is not suitable.')
        finalerr = -1; % Failure
    end
    n = n + 1;

end
end