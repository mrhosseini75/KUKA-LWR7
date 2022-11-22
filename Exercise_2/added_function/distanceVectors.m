%% Distance Vectores Function
% computes the distance vectors which are used in recursive N-E algorithm
% input:
%           robot(struct)
% output:
%           robot(struct)
%
function robot = distanceVectors(robot)

    % distance vector between point O and point CoJ1
    robot.link(1).r_ij = [0 0 0]';

    % distance vector between point CoJ1 and point CoJ2
    CoJ1 = robot.link(1).oTi(1:3,4);
    CoJ2 = robot.link(2).oTi(1:3,4);
    robot.link(2).r_ij = CoJ2 - CoJ1;

    % distance vector between point CoJ2 and point CoJ3
    CoJ3 = robot.link(3).oTi(1:3,4);
    robot.link(3).r_ij = CoJ3 - CoJ2;

    % distance vector between point CoJ1 and point CoM1
    CoM1 = robot.link(1).oTi * [0; robot.link(1).length/2; 0; 1];
    CoM1 = CoM1(1:3);
    robot.link(1).r_ci = CoM1 - CoJ1;

    % distance vector between point CoJ2 and point CoM2
    CoM2 = robot.link(2).oTi * [0; 0; robot.link(2).length/2; 1];
    CoM2 = CoM2(1:3);
    robot.link(2).r_ci = CoM2 - CoJ2;

    % distance vector between point CoJ3 and point CoM3
    CoM3 = robot.link(3).oTi * [robot.link(3).length/2; 0; 0; 1];
    CoM3 = CoM3(1:3);
    robot.link(3).r_ci = CoM3 - CoJ3;

    % distance vector between point CoM1 and point CoJ2
    robot.link(1).r_ipc = CoJ2 - CoM1;

    % distance vector between point CoM2 and point CoJ3
    robot.link(2).r_ipc = CoJ3 - CoM2;

    % distance vector between point CoM2 and E.E.
    E = robot.link(3).oTi * [robot.link(3).length; 0; 0; 1];
    E = E(1:3);
    robot.link(3).r_ipc = E - CoM3;
end