%%Test File
clc                                          % Clearing command window,figures
clear all
clf
%Before running the simulation, please find and run the script
%'startup_rvc'
set(0,'DefaultFigureWindowStyle','docked')   % Docking simulation beside code

%Workspace and Scaling
workspace = [-6 6 -6 6 0 5];             % Scaling workspace to fit the robots 
P1 = [ 0.3, 0.8,  0.0]; 
P2 = [ 5.0, 0.8,  0.0]; 
P3 = [ -5.0, 0.8,  0.0]; 
P4 = [ -4.0, -2.0,  0.0]; 
build = createEnvironment(workspace);
placeObjects(build,P1, P2, P3, P4);

