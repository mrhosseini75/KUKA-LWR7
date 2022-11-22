%% Rotation Matrix Funciton
% inputs: 
%           axis: rotation axis ('x', 'y' or 'z')
%           angle: rotation angle
% output:
%           iRj: the rotation matix between frame <i-1> and <i>
%
function iRj = rotationMatrix(axis, angle)
    % roll
    if axis == 'x'
        iRj = [1 0 0; 0 cos(angle) -sin(angle); 0 sin(angle) cos(angle)];   
    
    % pitch
    elseif axis == 'y'
    	iRj = [cos(angle) 0 sin(angle); 0 1 0; -sin(angle) 0 cos(angle)];
  
    % yaw
    elseif axis == 'z'
        iRj = [cos(angle) -sin(angle) 0; sin(angle) cos(angle) 0; 0 0 1];
    end
end