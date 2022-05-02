% clear all;
% 
% robot = Cyton300e;
% q = zeros(1,7);
% scale = 0.5;
% robot.plot(q,'workspace',[0 4 0 4 -0.1 4],'scale',0.5);
% 
% robot.teach;

clear all

robot = DOBOT;
q = zeros(1,4);
scale = 0.5;
% robot.plot(q,'workspace',[0 4 0 4 -0.1 4],'scale',0.5);

robot.model.teach;




