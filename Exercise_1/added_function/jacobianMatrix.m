%% Jacobian Matrix Function 
% inputs:
%           oTi: a tensor with size 4x4xn (n = number of joints) containing transoframtion matrices for each joint's frame w.r.t <o> (inertial refrence frame) 
%           i: the frame number for which the jacobian matrix is desired to be computed
%           joints_type: a vector with size n (number of joints) containing the type of each joint (revolute=0 or prismatic=1)
%          
% output:
%           J = jacobian matrix for frame <i> w.r.t <o>
%
function J = jacobianMatrix(oTi, i, joints_type)    
    J = zeros(6, length(joints_type));
    
    for j = 1:length(joints_type)
        
        % revolute joint
        if(joints_type(j) == 0)
            
            if j <= i
                %angular jacobian 
                k_j = oTi(1:3,3,j);
                J(1:3, j) = k_j;

                %linear jacobian 
                A = oTi(1:3,4,i);
                B = oTi(1:3,4,j);
                r_ij = A - B;
                J(4:6, j) = cross(k_j, r_ij);
            end     

        % primsatic joint
        elseif(joints_type(j) == 1)

            %angular jacobian 
            J(1:3,j) = zeros(3,1);

            if j <= i
                %linear jacobian 
                k_j = oTi(1:3,3,j);
                J(4:6,j) = k_j;
            end

        end
        
    end
    
end