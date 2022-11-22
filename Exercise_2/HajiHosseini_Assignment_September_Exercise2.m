%% System initiation
clc;
clear all;
close all;
addpath('added_function');
%% Exercise2-Recursive inverse dynamics
% gravity and external forces and torques effect
g = [0 -9.81 0]';
F_ext = zeros(3);
M_ext = zeros(3);

% joints type
% revolute case = 0
% prismatic case = 1
robot.link(1).joint_type = 0; 
robot.link(2).joint_type = 1; 
robot.link(3).joint_type = 0; 

% links mass
robot.link(1).mass = 10; %kg
robot.link(2).mass = 22; %kg
robot.link(3).mass = 6.5;%kg

% links length
robot.link(1).length = 0.3; %m
robot.link(2).length = 1.2; %m
robot.link(3).length = 0.3; %m

% inertial moments
robot.link(1).inertia = [0 0 0; 0 0 0; 0 0 0.4]; %kg/m^2
robot.link(2).inertia = [0 0 0; 0 0 0; 0 0 0.7]; %kg/m^2
robot.link(3).inertia = [0 0 0; 0 0 0; 0 0 0.2]; %kg/m^2

% transformation matrix between frame <i-1> and <i> for initial configuration
robot.link(1).iTj_init = eye(4);
robot.link(2).iTj_init = eye(4);
robot.link(2).iTj_init(1:3,1:3) = rotationMatrix('y', pi/2);
robot.link(2).iTj_init(2,4) = robot(1).link(1).length;
robot.link(3).iTj_init = eye(4);
robot.link(3).iTj_init(1:3,1:3) = rotationMatrix('y', -pi/2);
robot.link(3).iTj_init(3,4) = robot(1).link(2).length;

% transformation matrix between frame <o> and <i> for inital configuration
for i = 1:length(robot.link)
    oTi_init_aux = eye(4);
    for j = 1:i
        oTi_init_aux = oTi_init_aux * robot.link(j).iTj_init; 
    end
    robot.link(i).oTi_init = oTi_init_aux;
end
%% Exercise2_1
% trajectory snapshot
robot.link(1).q = pi/4; %rad
robot.link(2).q = -0.3; %m
robot.link(3).q = pi/2; %rad
robot.link(1).qdot = 0.4; %rad/s
robot.link(2).qdot = 0.1; %m/s
robot.link(3).qdot = 0.2; %rad/s
robot.link(1).qdotdot = 0.2; %rad/s^2
robot.link(2).qdotdot = 0.4; %m/s^2
robot.link(3).qdotdot = 0; %rad/s^2

% change robot configuration
robot = changeConfig(robot);

% computation of distance vectors
robot = distanceVectors(robot);

% computation of joints torque using recursive N-E algorithm
disp("Exercise2_1:")
tau = recursiveNewtonEuler(robot, g, F_ext, M_ext)
disp("------------")
%% Exercise2_2
% trajectory snapshot
robot.link(1).q = 0;          %rad
robot.link(2).q = -0.6;       %m
robot.link(3).q = 0;          %rad
robot.link(1).qdot = 0.2;     %rad/s
robot.link(2).qdot = 0.4;     %m/s
robot.link(3).qdot = -0.3;     %rad/s
robot.link(1).qdotdot = 0.1;  %rad/s^2
robot.link(2).qdotdot = 0;    %m/s^2
robot.link(3).qdotdot = -0.04; %rad/s^2

% change robot configuration
robot = changeConfig(robot);

% computation of distance vectors
robot = distanceVectors(robot);

% computation of joints torque using recursive N-E algorithm
disp("Exercise2_2:")
tau = recursiveNewtonEuler(robot, g, F_ext, M_ext)
disp("------------")
%% Exercise2_3
% trajectory snapshot
robot.link(1).q = pi/6;     %rad
robot.link(2).q = -0.45;    %m
robot.link(3).q = -pi/2;     %rad
robot.link(1).qdot = 0.2;   %rad/s
robot.link(2).qdot = 1;     %m/s
robot.link(3).qdot = -1;     %rad/s
robot.link(1).qdotdot = 0.1;%rad/s^2
robot.link(2).qdotdot = -0.3;%m/s^2
robot.link(3).qdotdot = -0.6; %rad/s^2

% change robot configuration
robot = changeConfig(robot);

% computation of distance vectors
robot = distanceVectors(robot);

% computation of joints torque using recursive N-E algorithm
disp("Exercise2_3:")
tau = recursiveNewtonEuler(robot, g, F_ext, M_ext)
disp("------------")