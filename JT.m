clc;clear; close all;

ur5 = ur5_interface();
tf_frame.get_tf_tree();
%%
gst1 = [0 -1 0 0.25;
        -1 0 0 0.60;
        0 0 -1 0.22;
        0 0 0 1];

[ theta1 ] = ur5InvKin( gst1 );

% Define target pose (g_{st4}) for point (4)
gst4 = [0 -1 0 0.25+0.0707;
        -1 0 0 0.60-0.0707;
        0 0 -1 0.22;
        0 0 0 1];

[ theta4 ] = ur5InvKin( gst4 );

% Define initial pose (g_{st1}) for point (1)
gst2 = [0 -1 0 0.25-0.0707/2;
        -1 0 0 0.60-0.0707/2;
        0 0 -1 0.22;
        0 0 0 1];

[ theta2 ] = ur5InvKin( gst2 );

% Define target pose (g_{st4}) for point (4)
gst3 = [0 -1 0 0.25+0.0707-0.0707/2;
        -1 0 0 0.60-0.0707-0.0707/2;
        0 0 -1 0.22;
        0 0 0 1];

[ theta3 ] = ur5InvKin( gst3 );

K = 0.2;
q_target1 = theta1(:,3);
finalerr = JTcontrol(gst1, q_target1, K);

