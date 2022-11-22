%% Skew Symmetric Form Function
% input:
%           x: a vector with size 3 
%
% output:
%           X: the skew-symmetric for of the input vector
%
function X = skew(x)
    X = [ 0    -x(3)   x(2); 
         x(3)    0    -x(1);
        -x(2)   x(1)    0 ];
end