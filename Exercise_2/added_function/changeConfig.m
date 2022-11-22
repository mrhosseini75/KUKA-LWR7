%% Change Robot Configuration Function
% changes robot configuration from initial state to the desired values
% input:
%          robot(struct)
% output:
%          robot(struct)
%
function robot = changeConfig(robot)
    % transformation matrix between frame <i-1> and <i> for desired configuration
    % frame <o> to <a>
    robot.link(1).iTj = transformationMatrix(robot.link(1).q, robot.link(1).iTj_init, robot.link(1).joint_type);
    % frame <a> to <b>
    robot.link(2).iTj = transformationMatrix(robot.link(2).q, robot.link(2).iTj_init, robot.link(2).joint_type);
    % frame <b> to <c>
    robot.link(3).iTj = transformationMatrix(robot.link(3).q, robot.link(3).iTj_init, robot.link(3).joint_type);

    % transformation matrix between frame <o> and <i> for desired configuration
    for i = 1:length(robot.link)
        oTi_aux = eye(4);
        for j = 1:i
            oTi_aux = oTi_aux * robot.link(j).iTj; 
        end
        robot.link(i).oTi = oTi_aux;
    end
end