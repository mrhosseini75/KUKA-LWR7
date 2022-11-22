%% Transformation Matrix Function
% inputs:
%           qi: configuration of joint i
%           iTj_init: transformation matrix between frame <i-1> and <i> at the initial configuration (q = 0)
%           joint_type: revolute=0 or prismatic=1
% output:
%           iTj: transformation matrix between frame <i-1> and <i> at the given configuration 
%
function iTj = transformationMatrix(qi, iTj_init, joint_type)
    iTj = eye(4);
    
    % revolute joint
    if joint_type == 0
        iTj(1:3,1:3) = rotationMatrix('z', qi);
        
    % prismatic joint
    elseif joint_type == 1
        iTj(3,4) = qi;  
    end
    
    iTj = iTj_init * iTj;
    
end