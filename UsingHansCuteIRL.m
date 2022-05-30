%% Running on real Hans Cute
% Draw Triangle
% Load .mat file with all Joint State Trajectories
load('drawTriangleTraj.mat');

% Setup Installation
rosinit('http://10.42.0.1:11311')   % For Raspberry PI
%rosinit('http://localhost:11311')  % For Native Linux

% Setup Variables
cute_enable_robot_client = rossvcclient('enableCyton');
cute_enable_robot_msg = rosmessage(cute_enable_robot_client);

% To enable the Robot
cute_enable_robot_msg.TorqueEnable = true;  % false to disable (hold on to arm when disabling
cute_enable_robot_client.call(cute_enable_robot_msg);

% Create a Subscriber to see current state of each joint
stateSub = rossubscriber('/joint_states');
receive(stateSub,2)
msg = stateSub.LatestMessage;
% View the joint angles and EE position
msg.JointAngles
msg.Pos

% Setup Publisher and Message to control Joint States
cute_position_publisher = rospublisher('/cyton_position_commands');
cute_position_msg = rosmessage(cute_position_publisher);

% Send qMatrix_1
for i = 1:10:size(qMatrix_1, 1)
    % Publish joint state
    cute_position_msg.Data = qMatrix_1(i, :);
    cute_position_publisher.send(cute_position_msg);
    %pause(0.25);   % Not sure if we need a pause here to wait until it
    %reaches the state?
end

% Send qMatrix_2
for i = 1:5:size(qMatrix_2, 1)
    % Publish joint state
    cute_position_msg.Data = qMatrix_2(i, :);
    cute_position_publisher.send(cute_position_msg);
    %pause(0.25);   % Not sure if we need a pause here to wait until it
    %reaches the state?
end

% Send qMatrix_3
for i = 1:5:size(qMatrix_3, 1)
    % Publish joint state
    cute_position_msg.Data = qMatrix_3(i, :);
    cute_position_publisher.send(cute_position_msg);
    %pause(0.25);   % Not sure if we need a pause here to wait until it
    %reaches the state?
end

% Send qMatrix_4
for i = 1:10:size(qMatrix_4, 1)
    % Publish joint state
    cute_position_msg.Data = qMatrix_4(i, :);
    cute_position_publisher.send(cute_position_msg);
    %pause(0.25);   % Not sure if we need a pause here to wait until it
    %reaches the state?
end