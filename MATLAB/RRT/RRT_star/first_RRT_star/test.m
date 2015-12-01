%Discrete LQR + Kalman Filter
clear
figure(1)
clf
m = 10;
cx = 5;
cy = 3;
kx = 0;
ky = 0;

Q = .02*eye(4);
R = .01*eye(2);

N = 5000;
tf = 15;
t = linspace(0, tf, N);
dt = t(2) - t(1);
r(:,1) = linspace(0, 8, N);
r(:,2) = linspace(0, 7, N);
r(:,3) = zeros(1,N);
r(:,4) = zeros(1,N);
r = r';


A_cont = [0 0 1 0; 0 0 0 1; kx/m 0 cx/m 0; 0 ky/m 0 cy/m];
B_cont =[0 0 1/m 0; 0 0 0 1/m]';
C_cont = [1 0 0 0; 0 1 0 0];
sys_cont = ss(A_cont, B_cont, C_cont, zeros(2,2));
sys_disc = c2d(sys_cont, dt);
A = sys_disc.a;
B = sys_disc.b;
C = sys_disc.c;


x_nominal = zeros(4,N);
x_nominal(:,1) = [0 0 0 0]';
x(:,1) = [0 0 0 0]';
x_hat(:,1) = [0 0 0 0]';
x_pred(:,1) = [0 0 0 0]';

sigma_pred = zeros(4,4,N);
sigma_update = zeros(4,4,N);
lambda = zeros(4,4,N);
u = zeros(2,N);

% Q_lqr = 5*eye(4);
Q_lqr = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
R_lqr = 0.1*eye(2);
[K_ss, ~, ~] = dlqr(A, B, Q_lqr, R_lqr); %K is lqr gain, P is solution to Riccatti eqn, E is eig(A-BK) of system
[L_ss, ~, ~] = dlqr(A', C', Q, R);

w = .01*randn(2, N);
v = .02*randn(2, N);
G = eye(2);
H = eye(2);


% % for tracking controller
P = zeros(4,4,N);
s = zeros(4,4,N);
Qf = [1 0 0 0; 0 1 0 0; 0 0 10 0; 0 0 0 10];
% s(:,N) = - Qf*r(:,N);
for k= N:-1:2
    P_dot= - P(:,:,k)*A - A'*P(:,:,k) + P(:,:,k)*B*inv(R_lqr)*B'*P(:,:,k)- Q_lqr;
    s_dot= (P(:,:,k)*B*inv(R)*B'- A')*s(:,k) + Q*r(:,k);
    P(:,:,k-1)= P(:,:,k) - P_dot*dt;
    s(:,k-1)= s(:,k) - s_dot*dt;
end
% 
x_hat_plus(:,1) = [10 9 0 0]';
P_plus(:,:,1) = zeros(4,4);
z(:,1) = [0 0]';
for k=2:N
   x_nominal(:,k) = A*x_nominal(:,k-1) + B*u(:,k-1);
   %trajectory control
    u(:,k) = -K_ss*(x_nominal(:,k) - r(:,k)) + G*v(:,k) - R\B'*s(:,k);
% %    u(:,k) = -K_ss*(x_hat_plus(:,k-1)) + G*v(:,k);
%         u(:,k) = -K_ss*(x(:,k-1)) + G*v(:,k);
        u_error(:,k) = -K_ss*x_hat_plus(:,k-1) + G*v(:,k-1);
%     %state estimation
%     %prediction step
%     x_pred(:,k) = A*x_hat(:,k-1) + B*u_error(:,k-1);
%     sigma_pred(:,:,k) = A*sigma_update(:,:,k-1)*A' + Q;
%     z_err(:,k) = C*x_pred(:,k) + H*w(:,k);  %Take a measurement, plus some noise.
%     
% %     %update step
%     S(:,:,k) = C*sigma_pred(:,:,k)*C' + R;
%     L(:,:,k) = sigma_pred(:,:,k)*C'/S(:,:,k);
%     x_hat(:,k) = x_pred(:,k) + L(:,:,k)*(z_err(:,k) - C*x_pred(:,k));
%     sigma_update(:,:,k) = sigma_pred(:,:,k) - L(:,:,k)*C*sigma_pred(:,:,k);
%     
%     lambda(:,:,k) = (A-B*K_ss)*lambda(:,:,k-1)*(A-B*K_ss)'+ L(:,:,k)*C*sigma_pred(:,:,k);
%     
%     x(:,k) = (x_nominal(:,k) + x_hat(:,k));

% %Regular Kalman Filter.
x_hat_minus(:,k) = A*x_hat_plus(:,k-1) + B*u(:,k-1);
P_minus(:,:,k) = A*P_plus(:,:,k-1)*A' + Q;
z(:,k) = C*x_nominal(:,k) + w(:,k);

K(:,:,k) = P_minus(:,:,k)*C'/(C*P_minus(:,:,k)*C' + R);
x_hat_plus(:,k) = x_hat_minus(:,k) + K(:,:,k)*(z(:,k) - C*x_hat_minus(:,k));
P_plus(:,:,k) = inv(P_minus(:,:,k));
    

end


subplot(2,2,1)
plot(t, x_hat_plus)
ylabel('x, r')
hold on
plot(t,r,'k:')
subplot(2,2,2)
plot(t,u)
ylabel('u')
hold on
plot([0;15],[0;0], 'k:')
subplot(2,2,3)
plot(t, x_nominal)
% plot(t, x_hat_plus)
hold on
% plot(t,r, ':')
ylabel('x_{hat}')
subplot(2,2,4)
% plot(r(1,:), r(2,:), 'k:', x_hat(1,:), x_hat(2,:))
plot(x(1,:), x(2,:))
% hold on
% plot
title('Position of robot')

hold on
for i = 1:100:N
%     plot(x(1,i), x(2,i), 'k:', x_hat(1,i), x_hat(2,i), 'rx')
    plot_gaussian_ellipsoid([x_hat_plus(1, i), x_hat_plus(2,i)], P_plus(1:2,1:2,i), 4, 25); % mean, std dev, stdwidth (2~= .86), n points. 
end
legend('estimated position', 'error ellipse', 'Location', 'NorthWest')
