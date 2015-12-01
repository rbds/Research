%Discrete LQR + Kalman Filter
clear
clf
m = 1;
cx = 2;
cy = 3;
kx = 0;
ky = 0;

% A = [0 0 1 0; 0 0 0 1; kx/m 0 cx/m 0; 0 ky/m 0 cy/m];
% B =[0 0 1/m 0; 0 0 0 1/m]';
% C = [1 0 0 0; 0 1 0 0];
A = eye(4);
B = eye(4);
C = eye(4);

%kalman filter noise constants
Q = .2*eye(4);
R = .1*eye(4);

N = 1000;
tf = 10;
t = linspace(0, 2*tf, 2*N);
dt = t(2) - t(1);
r(1,:) = linspace(0, 80, N);
r(2,:) = linspace(0, 70, N);
r(3,:) = zeros(1,N);
r(4,:) = zeros(1,N);

r(1, N+1:2*N) = linspace(80, 200, N);
r(2, N+1:2*N) = linspace(70, 120, N);
r(3,N+1:2*N) = zeros(1,N);
r(4,N+1:2*N) = zeros(1,N);

x_nominal = zeros(4,2*N);
x_nominal(:,1) = [0 0 0 0]';
x_hat(:,1) = [0 0 0 0]';
sigma_pred = zeros(4,4,2*N);

x_err_hat(:,1) = [0 0 0 0]';
x_err(:,1) = [0 0 0 0]';
sigma_update = zeros(4,4,2*N);
lambda = zeros(4,4,2*N);
u = zeros(4,N);

Q_lqr = 2*eye(4);
R_lqr = 0.25*eye(4);
[K_ss, P, E] = dlqr(A,B,Q_lqr,R_lqr); %K is lqr gain, P is solution to Ricca1ti eqn, E is eig(A-BK) of system
[L_ss, S, E] = dlqr(A', C', Q, R);

% P = [-5+5*j, -5-5*j, -2+2*j, -2-2*j];
% K = place(A,B,P);


w = .1*randn(4, 2*N);
v = abs(.3*randn(4, 2*N));
G = .5*eye(4);
H = eye(4);

for k=2:2*N
    
   %trajectory control
    u(:,k) = -K_ss*(x_nominal(:,k-1) - r(:,k-1)) +G*v(:,k);
    x_nominal(:,k) = A*x_nominal(:,k-1) + B*u(:,k-1);

    %state estimation
    %prediction step
    x_err(:,k) = A*x_err_hat(:,k-1) + B*u(:,k-1);
    sigma_pred(:,:,k) = A*sigma_update(:,:,k-1)*A' + Q;
    z_err(:,k) = C*x_err(:,k) + H*w(:,k);  %Take a measurement, plus some noise.
    
%     %update step
    S(:,:,k) = C*sigma_pred(:,:,k)*C' + R;
    L(:,:,k) = sigma_pred(:,:,k)*C'/S(:,:,k);
    x_err_hat(:,k) = x_err(:,k) + L(:,:,k)*(z_err(:,k) - C*x_err(:,k));
    sigma_update(:,:,k) = sigma_pred(:,:,k) - L(:,:,k)*C*sigma_pred(:,:,k);
    
    lambda(:,:,k) = (A-B*K_ss)*lambda(:,:,k-1)*(A-B*K_ss)'+ L(:,:,k)*C*sigma_pred(:,:,k);
    
    x_hat(:,k) = (x_nominal(:,k) + x_err_hat(:,k));
    
end


subplot(2,2,1)
plot(t,x_nominal, 'r:')
ylabel('x, r')
hold on
plot(t,r,'k')
subplot(2,2,2)
plot(t,u)
ylabel('u')
subplot(2,2,3)
plot(t, x_hat)
hold on
plot(t,r, ':')
ylabel('x_{hat}, r')
subplot(2,2,4)
plot(r(1,:), r(2,:), 'k:', x_hat(1,:), x_hat(2,:))
title('Position of robot')
legend('reference', 'estimated position', 'Location', 'NorthWest')
hold on
for i = 1:100:2*N
    plot(r(1,i), r(2,i), 'k:', x_hat(1,i), x_hat(2,i), 'rx')
    plot_gaussian_ellipsoid([r(1, i), r(2,i)], sigma_update(1:2,1:2,i) + lambda(1:2,1:2,i), 4, 25); % mean, std dev, stdwidth (2~= .86), n points. 
end
