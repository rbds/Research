%connect function
%takes as inputs x_a, x_b
%outputs state estimates, control inputs, and control law.

%needs to compute a collision free trajectory from x_a to x_b.
%This can be done for the linear system by an lqr. Or, for a dubins vehicle,
%the set of L-S-R curves

%output  should be a set of intermediate states, control input to get between states, and the LQR controller gain for each step 
function [path] = connect( x_a, x_b)

step_size = 1;
%for now, just use straight line paths.
    dist=norm(x_b - x_a);
    x_len = x_b(1)- x_a(1);
    y_len = x_b(2) - x_a(2);
    steps = floor(dist/step_size);
%control law is just straight line path.
    m = (x_len)/(y_len); % slope between two points.
    path(1).p = [x_a(1); x_a(2); 0];
    for i=2:steps
       path(i).p = [x_a(1) + x_len/steps*i; x_a(2) + y_len/steps*i; 0 ]; %#ok<AGROW> %[x y theta]
    end
    
%path.inputs = [];       %control input to get from path(n-1) to path(n)
%path.gains = [];        %controller gain for travelling from path(n-1) to path(n)
end
