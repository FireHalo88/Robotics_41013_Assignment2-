clear all
r = HansCute;
%r.plotModel
%r.plotModel
% workspace = [-4 4 -4 4 -4 4];                                       % Set the size of the workspace when drawing the robot
% 
% scale = 0.5;
% 
q = zeros(1,7);                                                     % Create a vector of initial joint angles

%r.plot()               % Plot the robot
% disp('Press enter to continue:');
% pause;
% q2 = [pi/2, pi/2, 0, 0, 0, 0, 0];
% r.robotModel.animate(q2);
%r.robotModel.teach();
r.plotModel();