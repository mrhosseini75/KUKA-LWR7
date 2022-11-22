%% System initiation
clc; 
clear all;
close all;
addpath('added_function');
%% Exercise1-Statics of open kinematic Chains
% parameters
m = [10 22 6.5];   %kg  mass = [link1 link2 link3]
l = [0.3 1.2 0.3]; %m lenght = [link-1 link2 link3]
joints_type = [0 1 0]; % (0: revolut joint) and (1: prismatic joint)
g = 9.81; %m/s^2 gravity

% transformation matrix between previous frame link <i-1> and present link <i> for initial configuration
% frame <o> to <a> 
iTj_init(:,:,1) = eye(4);
% frame <a> to <b>
iTj_init(:,:,2) = eye(4);
iTj_init(1:3,1:3,2) = rotationMatrix('y', pi/2);
iTj_init(2,4,2) = l(1);
% frame <b> to <c>
iTj_init(:,:,3) = eye(4);
iTj_init(1:3,1:3,3) = rotationMatrix('y', -pi/2);
iTj_init(3,4,3) = l(2);

% transformation matrix between frame <o> and <i> for inital configuration
for i = 1:length(joints_type)
    oTi_init_aux = eye(4);
    for j = 1:i
        oTi_init_aux = oTi_init_aux * iTj_init(:,:,j); 
    end
    oTi_init(:,:,i) = oTi_init_aux;
end
%% Exercise1_1
% desired configuration
q = [0 0 0];

% transformation matrix between frame <i-1> and <i> for desired configuration
% frame <o> to <a>
iTj(:,:,1) = transformationMatrix(q(1), iTj_init(:,:,1), joints_type(1));
% frame <a> to <b>
iTj(:,:,2) = transformationMatrix(q(2), iTj_init(:,:,2), joints_type(2));
% frame <b> to <c>
iTj(:,:,3) = transformationMatrix(q(3), iTj_init(:,:,3), joints_type(3));

% transformation matrix between frame <o> and <i> for desired configuration
for i = 1:length(joints_type)
    oTi_aux = eye(4);
    for j = 1:i
        oTi_aux = oTi_aux * iTj(:,:,j); 
    end
    oTi(:,:,i) = oTi_aux;
end

% jacobian matrix for frame <i> w.r.t frame <o>
for i = 1:length(joints_type)
    J(:,:,i) = jacobianMatrix(oTi, i, joints_type);
end
% CoMi = Center of Mass link<i>
% CoJi = Center of Joint link <i>
% distance vector between point COJ1 and point CoM1
CoM1 = oTi_init(:,:,1)*[0; l(1)/2; 0; 1];
CoM1 = CoM1(1:3);
CoJ1 = oTi_init(1:3,4,1);
rc1 = CoM1 - CoJ1;

% distance vector between point CoJ2 and point CoM2
CoM2 = oTi_init(:,:,2)*[0; 0; l(2)/2; 1];
CoM2 = CoM2(1:3);
CoJ2 = oTi_init(1:3,4,2);
rc2 = CoM2 - CoJ2;

% distance vector between point CoJ3 and point CoM3
CoM3 = oTi_init(:,:,3)*[l(3)/2; 0; 0; 1];
CoM3 = CoM3(1:3);
CoJ3 = oTi_init(1:3,4,3);
rc3 = CoM3 - CoJ3;

% rigid body jacobian for link i w.r.t frame <o>
Sc(:,:,1) = [eye(3) zeros(3); -skew(rc1) eye(3)];
Sc(:,:,2) = [eye(3) zeros(3); -skew(rc2) eye(3)];
Sc(:,:,3) = [eye(3) zeros(3); -skew(rc3) eye(3)];

% wrench vector acting on link i (only gravity effect)
for i = 1:length(joints_type)
    W(:,i) = [zeros(3,1); 0; -m(i)*g; 0];
end
% calculate joints torque in static equilibrium point
tau = zeros(length(joints_type), 1);
for i = 1:length(joints_type)
    tau = tau - (Sc(:,:,i) * J(:,:,i))' * W(:,i);
end
disp("Exercise1_1:")
tau
disp("------------")
%% Exercise 1_2
% desired configuration
q = [0 0 -pi/2];

% transformation matrix between frame <i-1> and <i> for desired configuration
% frame <o> to <a>
iTj(:,:,1) = transformationMatrix(q(1), iTj_init(:,:,1), joints_type(1));
% frame <a> to <b>
iTj(:,:,2) = transformationMatrix(q(2), iTj_init(:,:,2), joints_type(2));
% frame <b> to <c>
iTj(:,:,3) = transformationMatrix(q(3), iTj_init(:,:,3), joints_type(3));

% transformation matrix between frame <o> and <i> for desired configuration
for i = 1:length(joints_type)
    oTi_aux = eye(4);
    for j = 1:i
        oTi_aux = oTi_aux * iTj(:,:,j); 
    end
    oTi(:,:,i) = oTi_aux;
end

% jacobian matrix for frame <i> w.r.t frame <o>
for i = 1:length(joints_type)
    J(:,:,i) = jacobianMatrix(oTi, i, joints_type);
end
% CoMi = Center of Mass link<i>
% CoJi = Center of Joint link <i>
% distance vector between point CoJ1 and point CoM1
CoM1 = oTi(:,:,1)*[0; l(1)/2; 0; 1];
CoM1 = CoM1(1:3);
CoJ1 = oTi(1:3,4,1);
rc1 = CoM1 - CoJ1;

% distance vector between point CoJ2 and point CoM2
CoM2 = oTi(:,:,2)*[0; 0; l(2)/2; 1];
CoM2 = CoM2(1:3);
CoJ2 = oTi(1:3,4,2);
rc2 = CoM2 - CoJ2;

% distance vector between point CoJ3 and point CoM3
CoM3 = oTi(:,:,3)*[l(3)/2; 0; 0; 1];
CoM3 = CoM3(1:3);
CoJ3 = oTi(1:3,4,3);
rc3 = CoM3 - CoJ3;

% rigid body jacobian for link i w.r.t frame <o>
Sc(:,:,1) = [eye(3) zeros(3); -skew(rc1) eye(3)];
Sc(:,:,2) = [eye(3) zeros(3); -skew(rc2) eye(3)];
Sc(:,:,3) = [eye(3) zeros(3); -skew(rc3) eye(3)];

% wrench vector acting on link i (only gravity effect)
for i = 1:length(joints_type)
    W(:,i) = [zeros(3,1); 0; -m(i)*g; 0];
end
% calculate joints torque in static equilibrium point
tau = zeros(length(joints_type), 1);
for i = 1:length(joints_type)
    tau = tau - (Sc(:,:,i) * J(:,:,i))' * W(:,i);
end
disp("Exercise1_2:")
tau
disp("------------")
%% Exercise 1_3
% desired configuration
q = [-pi/4 0 -pi/4];

% transformation matrix between frame <i-1> and <i> for desired configuration
% frame <o> to <a>
iTj(:,:,1) = transformationMatrix(q(1), iTj_init(:,:,1), joints_type(1));
% frame <a> to <b>
iTj(:,:,2) = transformationMatrix(q(2), iTj_init(:,:,2), joints_type(2));
% frame <b> to <c>
iTj(:,:,3) = transformationMatrix(q(3), iTj_init(:,:,3), joints_type(3));

% transformation matrix between frame <o> and <i> for desired configuration
for i = 1:length(joints_type)
    oTi_aux = eye(4);
    for j = 1:i
        oTi_aux = oTi_aux * iTj(:,:,j); 
    end
    oTi(:,:,i) = oTi_aux;
end

% jacobian matrix for frame <i> w.r.t frame <o>
for i = 1:length(joints_type)
    J(:,:,i) = jacobianMatrix(oTi, i, joints_type);
end
% CoJi = Center of Joint link <i>
% distance vector between point CoJ3 and point P3
P3 = oTi(:,:,3)*[l(3); 0; 0; 1];
P3 = P3(1:3);
CoJ3 = oTi(1:3,4,3);
rp3 = P3 - CoJ3;

% wrench vector acting on link i (only ext. force acting on E.E.)
F_ext = [-0.7; 0.5; 0];
W = zeros(6,3);
W(:,3) = [cross(rp3, F_ext); F_ext];
% calculate joints torque in static equilibrium point
tau = zeros(length(joints_type), 1);
for i = 1:length(joints_type)
    tau = tau - (J(:,:,i))' * W(:,i);
end
disp("Exercise1_3:")
tau 
disp("------------")
%% Exercise 1_4
% desired configuration
q = [-pi/4 0 -pi/4];

% transformation matrix between frame <i-1> and <i> for desired configuration
% frame <o> to <a>
iTj(:,:,1) = transformationMatrix(q(1), iTj_init(:,:,1), joints_type(1));
% frame <a> to <b>
iTj(:,:,2) = transformationMatrix(q(2), iTj_init(:,:,2), joints_type(2));
% frame <b> to <c>
iTj(:,:,3) = transformationMatrix(q(3), iTj_init(:,:,3), joints_type(3));

% transformation matrix between frame <o> and <i> for desired configuration
for i = 1:length(joints_type)
    oTi_aux = eye(4);
    for j = 1:i
        oTi_aux = oTi_aux * iTj(:,:,j); 
    end
    oTi(:,:,i) = oTi_aux;
end

% jacobian matrix for frame <i> w.r.t frame <o>
for i = 1:length(joints_type)
    J(:,:,i) = jacobianMatrix(oTi, i, joints_type);
end
% CoMi = Center of Mass link<i>
% CoJi = Center of Joint link <i>
% distance vector between point CoJ1 and point CoM1
CoM1 = oTi(:,:,1)*[0; l(1)/2; 0; 1];
CoM1 = CoM1(1:3);
CoJ1 = oTi(1:3,4,1);
rc1 = CoM1 - CoJ1;

% distance vector between point CoJ2 and point CoM2
CoM2 = oTi(:,:,2)*[0; 0; l(2)/2; 1];
CoM2 = CoM2(1:3);
CoJ2 = oTi(1:3,4,2);
rc2 = CoM2 - CoJ2;

% distance vector between point CoJ3 and point CoM3
CoM3 = oTi(:,:,3)*[l(3)/2; 0; 0; 1];
CoM3 = CoM3(1:3);
CoJ3 = oTi(1:3,4,3);
rc3 = CoM3 - CoJ3;

% rigid body jacobian for link i w.r.t frame <o>
Sc(:,:,1) = [eye(3) zeros(3); -skew(rc1) eye(3)];
Sc(:,:,2) = [eye(3) zeros(3); -skew(rc2) eye(3)];
Sc(:,:,3) = [eye(3) zeros(3); -skew(rc3) eye(3)];

% distance vector between point CoM3 and point P3
P3 = oTi(:,:,3) * [l(3); 0; 0; 1];
P3 = P3(1:3);
CoM3 = oTi(:,:,3) * [l(3)/2; 0; 0; 1];
CoM3 = CoM3(1:3);
r_p3_c3 = P3 - CoM3;

% wrench vector acting on link i (gravity effect plus ext. force effect acting on E.E.)
F_ext = [-0.7; 0.5; 0];
for i = 1:length(joints_type)
    W(:,i) = [zeros(3,1); 0; -m(i)*g; 0];
end
W(:,3) = W(:,3) + [cross(r_p3_c3, F_ext); F_ext];
% calculate joints torque in static equilibrium point
tau = zeros(length(joints_type), 1);
for i = 1:length(joints_type)
    tau = tau - (Sc(:,:,i) * J(:,:,i))' * W(:,i);
end
disp("Exercise1_4:")
tau 
disp("------------")
%% Exercise1_5
% desired configuration
q = [0 0 0];

% transformation matrix between frame <i-1> and <i> for desired configuration
% frame <o> to <a>
iTj(:,:,1) = transformationMatrix(q(1), iTj_init(:,:,1), joints_type(1));
% frame <a> to <b>
iTj(:,:,2) = transformationMatrix(q(2), iTj_init(:,:,2), joints_type(2));
% frame <b> to <c>
iTj(:,:,3) = transformationMatrix(q(3), iTj_init(:,:,3), joints_type(3));

% transformation matrix between frame <o> and <i> for desired configuration
for i = 1:length(joints_type)
    oTi_aux = eye(4);
    for j = 1:i
        oTi_aux = oTi_aux * iTj(:,:,j); 
    end
    oTi(:,:,i) = oTi_aux;
end

% jacobian matrix for frame <i> w.r.t frame <o>
for i = 1:length(joints_type)
    J(:,:,i) = jacobianMatrix(oTi, i, joints_type);
end
% CoMi = Center of Mass link<i>
% CoJi = Center of Joint link <i>
% distance vector between point CoJ1 and point P1
P1 = oTi(:,:,1) * [0; l(1)/2; 0; 1];
P1 = P1(1:3);
CoJ1 = oTi(1:3,4,1);
rp1 = P1 - CoJ1;

% distance vector between point CoJ2 and point P2
P2 = oTi(:,:,2) * [0; 0; 2*l(2)/3; 1];
P2 = P2(1:3);
CoJ2 = oTi(1:3,4,2);
rp2 = P2 - CoJ2;

% wrench vector acting on link i (only external forces and the pure momntum effect)
F_ext1 = [-0.3; 0; 0];%N
F_ext2 = [0; 0.8; 0]; %N
T_ext = [0; 0; 1.2];  %N.m
W(:,1) = [cross(rp1, F_ext1); F_ext1];
W(:,2) = [cross(rp2, F_ext2); F_ext2];
W(:,3) = [T_ext; zeros(3,1)];

% calculate joints torque in static equilibrium point
tau = zeros(length(joints_type), 1);
for i = 1:length(joints_type)
    tau = tau - (J(:,:,i))' * W(:,i);
end
disp("Exercise1_5:")
tau
disp("------------")