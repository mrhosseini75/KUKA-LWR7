%% Recursive Newton Euler Function
% inputs:
%           robot(struct): contains data for:
%                               1. joints type
%                               2. links mass
%                               3. links length
%                               4. links inertia
%                               5. links frame transformation matrices
%                               6. distance vectors
%                               7. trajectory snapshot
%
%           gravity: effect of gravity force
%           F_ext: effect of external forces
%           M_ext: effect of external moments
%
% output:
%           inverse dynamic joints torque for the provided trajectory snapshot
%
function tau = recursiveNewtonEuler(robot, gravity, force_external, moment_external)
    
    % ----------------------   forward recursion  -------------------------
    
    n = length(robot.link); %number of links
    w_io = zeros(3,n+1); %rad/s (angular velocity of frame <i> w.r.t <o>)
    v_io = zeros(3,n+1); %m/s (linear velocity of frame <i> w.r.t <o>)
    wdot_io = zeros(3,n+1); %rad/s^2 (angular acceleration of frame <i> w.r.t <o>)
    vdot_io = zeros(3,n+1); %m/s^2 (linear acceleration of frame <i> w.r.t <o>)
    vdot_cio = zeros(3,n); %m/s^2 (linear acceleration of frame on point CoM_i  w.r.t <o>)
    
    for i=1:length(robot.link)

        % k axis of frame <i>
        k_i = robot.link(i).oTi(1:3,3);

        % revolute joint
        if robot.link(i).joint_type == 0

            % computation of linear and angular velocity
            w_io(:,i+1) = w_io(:,i) + k_i*robot.link(i).qdot;
            v_io(:,i+1) = v_io(:,i) + cross (w_io(:,i), robot.link(i).r_ij);
            
            % computation of linear and angular acceleration
            wdot_io(:,i+1) = wdot_io(:,i) + cross (w_io(:,i), k_i)*robot.link(i).qdot + k_i*robot.link(i).qdotdot;
            vdot_io(:,i+1) = vdot_io(:,i) + cross (wdot_io(:,i), robot.link(i).r_ij) + ...
                                cross(w_io(:,i), cross(w_io(:,i), robot.link(i).r_ij));
        end

        % prismatic joint
        if robot.link(i).joint_type == 1

            % computation of linear and angular velocity
            w_io(:,i+1) = w_io(:,i);
            v_io(:,i+1) = v_io(:,i) + cross (w_io(:,i), robot.link(i).r_ij) + ...
                        + k_i*robot.link(i).qdot + k_i*robot.link(i).qdot;

            % computation of linear and angular acceleration
            wdot_io(:,i+1) = wdot_io(:,i);
            vdot_io(:,i+1) = vdot_io(:,i) + cross (wdot_io(:,i), robot.link(i).r_ij) + ...
                                cross(w_io(:,i), cross(w_io(:,i), robot.link(i).r_ij)) + ...
                                2*(cross(wdot_io(:,i), k_i)*robot.link(i).qdot) + k_i*robot.link(i).qdotdot;
        end

        % linear acceleration of frame on point CoM_i
        vdot_cio(:,i) = vdot_io(:,i+1) + cross(wdot_io(:,i+1), robot.link(i).r_ci) + ...
                        cross(w_io(:,i+1),cross(w_io(:,i+1), robot.link(i).r_ci));
    end
    
    % ----------------------   backward recursion  -------------------------
    
    D = zeros(3,n);
    delta = zeros(3,n);
    force_ij = zeros(3,n+1);%N (force due to interaction of frame <i> and <i+1>)
    moment_ij = zeros(3,n+1);%N.m (moment due to interaction of frame <i> and <i+1>)
    
    for i=length(robot.link):-1:1
        
        % k axis of frame <i>
        k_i = robot.link(i).oTi(1:3,3);
        
        % projection of inertial operator for link i on frame <o>
        inertial_operator = robot.link(i).oTi(1:3,1:3) * robot.link(i).inertia * robot.link(i).oTi(1:3,1:3)';
        
        % computation of D_i and delta_i
        D(:,i) = robot.link(i).mass * vdot_cio(:,i);
        delta(:,i) = inertial_operator * wdot_io(:,i+1) + cross(w_io(:,i+1), (inertial_operator * w_io(:,i+1)));

        % computation of force and moment due to interaction of frame <i> and <i-1>
        force_ij(:,i) = force_ij(:,i+1) - robot.link(i).mass * gravity - force_external(:,i) + D(:,i);
        moment_ij(:,i) = moment_ij(:,i+1) - moment_external(:,i) - cross(-robot.link(i).r_ci, force_ij(:,i)) + ...
                        cross(robot.link(i).r_ipc, force_ij(:,i+1)) + delta(:,i);

        % computation of actuation torque of the joint i
        % revolute joint 
        if robot.link(i).joint_type == 0
            tau(:,i) = k_i' * moment_ij(:,i);
        
        % prismatic joint
        elseif robot.link(i).joint_type == 1
            tau(:,i) = k_i' * force_ij(:,i);
        end

    end
    
end