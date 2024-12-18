

clear all

ur5 = ur5_interface();

tf_frame.get_tf_tree();

ur5.switch_to_ros_control();
% 
% time_interval = 10; 

%% Run on UR5
% teach start position
ur5.switch_to_pendant_control();

disp('Please move the UR5 to the start position in freedrive mode, then press any key in the MATLAB figure window to continue...');

waitforbuttonpress;

q1=ur5.get_current_joints();

ur5.switch_to_ros_control();

disp('UR5 the positions of the start is taught');
disp( q1);

% teach target locations
ur5.switch_to_pendant_control();

disp('Please move the UR5 to the target position in freedrive mode, then press any key in the MATLAB figure window to continue...');

waitforbuttonpress;

q2=ur5.get_current_joints();

ur5.switch_to_ros_control();

disp('UR5 the positions of the target is taught');
disp(q2);

%% Simulation RIVZ

ur5.switch_to_ros_control();
% q1=[-1.6800,-2.2638,-1.4827,-0.9606,1.5698,-0.0281]';
% q2=[-1.6712, -2.0845,-1.7937,-0.8270,1.5702,-0.0188]';
qbase=[0,pi/2,0,pi/2,0,0]';
q12=q1+qbase;
q22=q2+qbase;
gst=ur5FwdKin3(qbase);
gst1=ur5FwdKin3(q12);
gst4=ur5FwdKin3(q22);
% % Define initial pose (g_{st1}) for point (1)
% gst1 = [0  -1   0  0.25;
%        -1   0   0  0.60;
%         0    0  -1  0.22;
%         0    0   0  1];
% 
% % Define target pose (g_{st4}) for point (4)
% gst4 = [0  -1   0  0.25+0.0707;
%        -1   0   0  0.60-0.0707;
%         0    0  -1  0.22;
%         0    0   0  1];
    
[d,gst2,gst3] = get_points(gst1,gst4);    
% Define initial pose (g_{st1}) for point (2)
% gst2 = [0  -1   0  0.25-0.0707/2;
%        -1   0   0  0.60-0.0707/2;
%         0    0  -1  0.22;
%         0    0   0  1];
gst22 = gst2;
gst22(3,4)= gst22(3,4)+0.05;
% Define target pose (g_{st4}) for point (3)
% gst3 = [0  -1   0  0.25+0.0707-0.0707/2;
%        -1   0   0  0.60-0.0707-0.0707/2;
%         0    0  -1  0.22;
%         0    0   0  1];
gst32 = gst3;
gst32(3,4)= gst32(3,4)+0.05;
%%
Frame_1=tf_frame('base_link','Frame_1',gst1);
Frame_2=tf_frame('base_link','Frame_2',gst2);
Frame_3=tf_frame('base_link','Frame_3',gst3);
Frame_4=tf_frame('base_link','Frame_4',gst4);

%% Denavit-Hartenberg parameters for` UR5
% d1 = 0.089159;
% d2 = 0;
% d3 = 0;
% d4 = 0.10915;
% d5 = 0.09465;
% d6 = 0.0823;
% 
% a1 = 0;
% a2 = -0.425;
% a3 = -0.39225;
% a4 = 0;
% a5 = 0;
% a6 = 0;
% 
% alpha1 = pi/2;
% alpha2 = 0;
% alpha3 = 0;
% alpha4 = pi/2;
% alpha5 = -pi/2;
% alpha6 = 0;
% 
% % Combine DH parameters into a table for use in forward/inverse kinematics
% DH_params = [ ...
%     alpha1, a1, d1, 0;
%     alpha2, a2, d2, 0;
%     alpha3, a3, d3, 0;
%     alpha4, a4, d4, 0;
%     alpha5, a5, d5, 0;
%     alpha6, a6, d6, 0];


% Compute inverse kinematics for gst1, gst2, gst3, and gst4
theta1_all = ur5InvKin(gst1);
theta2_all = ur5InvKin(gst2);
theta22_all = ur5InvKin(gst22);
theta3_all = ur5InvKin(gst3);
theta33_all = ur5InvKin(gst32);
theta4_all = ur5InvKin(gst4);

%%
theta1_valid=verify_theta(theta1_all);
theta2_valid=verify_theta(theta2_all);
theta3_valid=verify_theta(theta3_all);
theta4_valid=verify_theta(theta4_all);
theta22_valid=verify_theta(theta22_all);
theta33_valid=verify_theta(theta33_all);

% Get Q_start1 with minimal norm
norm_target1_point = zeros(size(theta1_valid,2),1);
for k = 1:size(theta1_valid,2)
    norm_target1_point(k) = norm(theta1_valid(:,k)-ur5.home);
end
[a, indx1] = min(norm_target1_point);
theta_target1 = theta1_valid(:,indx1);

% Get Q_start1 with minimal norm
norm_target2_point = zeros(size(theta2_valid,2),1);
for k = 1:size(theta2_valid,2)
    norm_target2_point(k) = norm(theta2_valid(:,k)-theta_target1);
end
[a, indx2] = min(norm_target2_point);
theta_target2 = theta2_valid(:,indx2);

% Get Q_start1 with minimal norm
norm_target3_point = zeros(size(theta3_valid,2),1);
for k = 1:size(theta3_valid,2)
    norm_target3_point(k) = norm(theta3_valid(:,k)-theta_target2);
end
[a, indx3] = min(norm_target3_point);
theta_target3 = theta3_valid(:,indx3);

% Get Q_start1 with minimal norm
norm_target4_point = zeros(size(theta4_valid,2),1);
for k = 1:size(theta4_valid,2)
    norm_target4_point(k) = norm(theta4_valid(:,k)-theta_target3);
end
[a, indx4] = min(norm_target4_point);
theta_target4 = theta4_valid(:,indx4);

% Get Q_start22 with minimal norm
norm_target22_point = zeros(size(theta22_valid,2),1);
for k = 1:size(theta4_valid,2)
    norm_target22_point(k) = norm(theta22_valid(:,k)-theta_target2);
end
[a, indx22] = min(norm_target22_point);
theta_target22 = theta4_valid(:,indx22);

% Get Q_start1 with minimal norm
norm_target33_point = zeros(size(theta33_valid,2),1);
for k = 1:size(theta33_valid,2)
    norm_target33_point(k) = norm(theta33_valid(:,k)-theta_target3);
end
[a, indx33] = min(norm_target33_point);
theta_target33 = theta4_valid(:,indx33);
%% Select valid solutions for all points
theta1 = theta_target1; % Use the first solution
theta2 = theta_target2; % Use the first solution
theta3 = theta_target3; % Use the first solution
theta4 = theta_target4; % Use the first solution
theta22 = theta_target22; % Use the first solution
theta33 = theta_target33; % Use the first solution


%% IK-based
disp('gst for point (2):');
disp(gst2);
disp('theta for point (2):');
disp(theta2');

disp('gst for point (3):');
disp(gst3);
disp('theta for point (3):');
disp(theta3');

disp('gst for point (4):');
disp(gst4);
disp('theta for point (4):');
disp(theta4');

% Visualize the motion in RViz using ur5.move_joints
disp('Visualizing motion using ur5.move_joints()...');
time_interval = 5; % Time interval for each motion
% Assuming 'ur5' is your robot object
ur5.move_joints(ur5.home, 10); % Move to point (1)
pause(10); % Wait for the motion to complete

% Assuming 'ur5' is your robot object
ur5.move_joints(theta1, 10); % Move to point (1)
pause(10); % Wait for the motion to complete

ur5.move_joints(theta2, time_interval); % Move to point (2)
pause(time_interval); % Wait for the motion to complete

ur5.move_joints(ur5.home, 10); % Move to point (2)
pause(10); % Wait for the motion to complete

ur5.move_joints(theta3, 10); % Move to point (2)
pause(10); % Wait for the motion to complete

ur5.move_joints(theta4, time_interval); % Move to point (4)
pause(time_interval); % Wait for the motion to complete

% Assuming 'ur5' is your robot object
ur5.move_joints(ur5.home, 10); % Move to point (1)
pause(10); % Wait for the motion to complete





%% RR-based
ur5.move_joints(ur5.home, 10); % Move to point (1)
pause(10); % Wait for the motion to complete

time_interval = 10;
K = 0.25; % Controller gain
theta1_all = ur5InvKin(gst1);

theta1 = theta1_all(:, 3);
ur5.move_joints(theta1, time_interval);
pause(time_interval); 
% Move to point 1
disp('Moving to point 1...');
finalerr1 = ur5RRcontrol(gst1, K, ur5);

% Move to point 2
disp('Moving to point 2...');
finalerr2 = ur5RRcontrol(gst2, K, ur5);

% Move to point 3
disp('Moving to point 3...');
finalerr3 = ur5RRcontrol(gst22, K, ur5);

% Move to point 3
disp('Moving to point 3...');
finalerr3 = ur5RRcontrol(gst32, K, ur5);



% Move to point 3
disp('Moving to point 3...');
finalerr3 = ur5RRcontrol(gst3, K, ur5);

% Move to point 4
disp('Moving to point 4...');
finalerr4 = ur5RRcontrol(gst4, K, ur5);

% Display final errors
disp(['Final error at point 1: ', num2str(finalerr1), ' cm']);
disp(['Final error at point 2: ', num2str(finalerr2), ' cm']);
disp(['Final error at point 3: ', num2str(finalerr3), ' cm']);
disp(['Final error at point 4: ', num2str(finalerr4), ' cm']);


%% TJ-based
ur5.move_joints(ur5.home, 10); % Move to point (1)
pause(10); % Wait for the motion to complete

time_interval = 10;
K = 2.3; % Controller gain
theta1_all = ur5InvKin(gst1);

theta1 = theta1_all(:, 3);ur5.move_joints(theta1, time_interval);
pause(time_interval); 
% Move to point 1
disp('Moving to point 1...');
finalerr1 = ur5JTcontrol(gst1, K, ur5);

% Move to point 2
disp('Moving to point 2...');
finalerr2 = ur5JTcontrol(gst2, K, ur5);

% Move to point 3
disp('Moving to point 3...');
finalerr3 = ur5JTcontrol(gst22, K, ur5);

% Move to point 3
disp('Moving to point 3...');
finalerr3 = ur5JTcontrol(gst32, K, ur5);

% Move to point 3
disp('Moving to point 3...');
finalerr3 = ur5JTcontrol(gst3, K, ur5);

% Move to point 4
disp('Moving to point 4...');
finalerr4 = ur5JTcontrol(gst4, K, ur5);

% Display final errors
disp(['Final error at point 1: ', num2str(finalerr1), ' cm']);
disp(['Final error at point 2: ', num2str(finalerr2), ' cm']);
disp(['Final error at point 3: ', num2str(finalerr3), ' cm']);
disp(['Final error at point 4: ', num2str(finalerr4), ' cm']);


%% drawing heart
% Example usage
center = gst1(1:3,4); % Initial center of the heart
scale = 0.004; % Scaling factor
num_points = 150; % Number of points
start_point = gst1(1:2,4); % Desired starting point

% Generate the heart curve
heart_points = generate_heart_curve(center, scale, num_points, start_point);
ur5.move_joints(ur5.home, 10);
pause(10);
ur5.move_joints(theta_target1, 10);
pause(10);
%%
time_interval = 0.5;

R = gst1(1:3, 1:3);
for i= 1:num_points
    gst_i= [R, heart_points(i,:)'; 0 0 0 1];
    theta_i_all = ur5InvKin(gst_i);
    theta_i = theta_i_all(:, 3);
    
    ur5.move_joints(theta_i, time_interval);
    pause(0.5);
end





function theta1_valid=verify_theta(theta1_all)
n=1;
for i=1:length(theta1_all)
    Q2 = theta1_all(:,i);
    % Guarantee the 2nd joint is above the table
    if Q2(2) > -pi && Q2(2) < 0
        % Calculate 4th joint config (to get height)
        g_joint4 = ur5FwdKin_3(Q2);
        % Guarantee the gripper is above the table
        if g_joint4(3,4) > 0.05
            theta1_valid(:,n) = Q2;
            n = n + 1;
        end
    end
end
end
