% This script will load a Dobot model with Teach functionality with
% assistance from Peter Corke's modified toolbox for Industrial Robotics
% subject 41013, UTS, Autumn 2022

% Clear any previous workspace
clc
clear all

% Load robot model
robot = DOBOT;

% Set initial joint angles
q = zeros(1,4);
scale = 0.5;

robot.model.teach;




