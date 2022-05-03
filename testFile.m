%%Test File
clc                                          % Clearing command window,figures
clear all
clf
%Before running the simulation, please find and run the script
%'startup_rvc'
set(0,'DefaultFigureWindowStyle','docked')   % Docking simulation beside code

%Workspace and Scaling
workspace = [-0.75 0.75 -0.75 0.75 0 0.75];             % Scaling workspace to fit the robots 
table = transl(0.0, 0.0, 0.0); 
pen1 = transl(0.10, -0.2, 0.273);
pen2 = transl(0.15, -0.2, 0.273);
pen3 = transl(0.20, -0.2, 0.273);
pen4 = transl(0.25, -0.2, 0.273);
canvas = transl(-0.3, -0.2, 0.22);
safetyBarrierPoint1 = transl(-0.47, -0.45, 0.0);
safetyBarrierPoint2 = transl(-0.47, 0.5, 0.0);

safetyBarrierPoint3 = transl(0.35, -0.45, 0.0);
safetyBarrierPoint4 = transl(0.35, 0.5, 0.0);

guard = transl(0.6, 0.57, 0.0);
fireExtinguisher = transl(0.3, 0.1, 0.57);
build = createEnvironment(workspace);
%Places all the objects (minus the Hans Cute Robot) into the environment
placeObjectsBetter(build, canvas, table,pen1, pen2,pen3, pen4, safetyBarrierPoint1,safetyBarrierPoint2, safetyBarrierPoint3,safetyBarrierPoint4, guard, fireExtinguisher);

%% Place Hans Cute Robot
hansCute_base = [0.275 0.0 0.2];
q = [0,0,0,0,0,0,0];
hansCute = HansCute("sup",workspace);
myRobot = hansCute.model;
hansCute.plotModel();
myRobot.base = transl(hansCute_base(1), hansCute_base(2), hansCute_base(3))*trotx(0)*troty(0)*trotz(0);
myRobot.plot(q, 'workspace', workspace, 'scale', 0.5);

