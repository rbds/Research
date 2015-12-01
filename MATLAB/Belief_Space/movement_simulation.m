function [ time, state] = movement_simulation (r)
v = 1;
t_final = norm(r(:, end) - r(:,1))/v; %dist/velocity
tspan = [0 t_final];
x_init = r(:,1)';
x_hat_init = x_init;
state_init = [x_init x_hat_init];
start = r(:,1);
done = r(:,end);

[time, state] = ode45(@(t, state) ode_eqn(t,state, start, done, t_final), tspan, state_init );



end

function [ state_dot ] = ode_eqn( t, state, start, done, t_final )
A = eye(2);
B=  eye(2);
C = [1 1];
R = [0.1 0; 0 0.05];
R2 = .1;
Q = .5*eye(2);
x = state(1:2,:);
x_hat = state(3:4, :);

% r_fun = @(t)  (done-start)*t/tspan;
r = (done-start).*t./t_final;

w = 0.1*randn(2,1);
v = 0.075*randn(2,1);

[K,S,E] = lqr(A,B,Q,R);
LT = lqr(A', C', Q, R2);
L = LT';

x_dot = A*x - B*K*(x_hat - r) + w;
x_hat_dot = A*x - B*K*(x_hat - r) + L*(C*x - C*x_hat);
state_dot = [x_dot; x_hat_dot];

end


