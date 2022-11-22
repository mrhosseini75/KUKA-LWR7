%% System initial 
clear all;
clc;
close all;
%% Exercise 3 – Dynamic Robot Control
% Imports rigid body tree of the robot and set the gravity
[kukaIiwa, kukaIiwa_info] = loadrobot('kukaIiwa7');
kukaIiwa.Gravity = [0,0,-9.81];

% Initial configuration of the robot
q0 = [0,0,0,0,0,0,0]';

% Delta q of the desired q*
dq0 = [pi/4, -pi/6, -pi/4, pi/3, -pi/2, pi/8, pi/4]';

% Desired q is the initial configuration plus the delta q
q_desired = q0 + dq0;
waypoints = [q0, q_desired];

open("KUKA_Iiwa_Subsystem_3a.slx");
open("KUKA_Iiwa_Subsystem_3b.slx");
