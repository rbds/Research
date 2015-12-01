%Discrete LQR + Kalman Filter
function [ x, uncert ] = estimation_plugin(estimator, start_node, end_node, N )

% clear
% figure(1)
% clf
% figure
%choose estimator type
% estimator = 'kalman';
% estimator = 'bry';
% estimator = 'sigma_points';
% estimator = 'particle_filter';
% global A B C;


m = 10;
cx = 3;
cy = 5;
kx = 0;
ky = 0;

Q = .02*eye(4); %noise covariances.
R = .01*eye(2);

% N = 1000;   %number of iterations to run.
tf = 15;

r(:,1) = linspace(start_node(1), end_node(1), N);
r(:,2) = linspace(start_node(2), end_node(2), N);
% r(:,1) = linspace(0, 80, N);     %reference trajectory
% r(:,2) = linspace(0, 70, N);
r(:,3) = zeros(1,N);
r(:,4) = zeros(1,N);
r = r';
% r(1, N+1:2*N) = linspace(80, 200, N);
% r(2, N+1:2*N) = linspace(70, 120, N);
% r(3,N+1:2*N) = zeros(1,N);
% r(4,N+1:2*N) = zeros(1,N);

N_pts = size(r,2);
t = linspace(0, tf, N_pts);
dt = t(2) - t(1);
A_cont = [0 0 1 0; 0 0 0 1; kx/m 0 cx/m 0; 0 ky/m 0 cy/m];      %linear system matrices.
B_cont =[0 0 1/m 0; 0 0 0 1/m]';
C_cont = [1 0 0 0; 0 1 0 0];
sys_cont = ss(A_cont, B_cont, C_cont, zeros(2,2));
sys_disc = c2d(sys_cont, dt);
A = sys_disc.a;
B = sys_disc.b;
C = sys_disc.c;

Q_lqr = 1*[10 0 0 0; 0 10 0 0; 0 0 0 0; 0 0 0 0];       %lqr cost function matrices.
R_lqr = 0.1*eye(2);
[K_ss, ~, ~] = dlqr(A, B, Q_lqr, R_lqr); %K is lqr gain, P is solution to Riccatti eqn, E is eig(A-BK) of system
% [L_ss, ~, ~] = dlqr(A', C', Q, R);

w = .01*randn(2, N_pts);        %gaussian noise
v = .02*randn(2, N_pts);
G = eye(2);
H = eye(2);

% % for tracking controller
P = zeros(4,4,N_pts);
s = zeros(4, N_pts);
Qf = [1 0 0 0; 0 1 0 0; 0 0 10 0; 0 0 0 10];
% s(:,N) = - Qf*r(:,N);
for k= N_pts:-1:2       %propogate the solution to the Riccatti eqn. backwards.
    P_dot= - P(:,:,k)*A - A'*P(:,:,k) + P(:,:,k)*B*inv(R_lqr)*B'*P(:,:,k)- Q_lqr;
    s_dot= (P(:,:,k)*B*inv(R)*B'- A')*s(:,k) + Q*r(:,k);
    P(:,:,k-1)= P(:,:,k) - P_dot*dt;
    s(:,k-1)= s(:,k) - s_dot*dt;
end

u = zeros(2,N);
    x_nominal = zeros(4,N_pts);
    x_nominal(:,1) = [r(1,1) r(2,1) 0 0]';
    z(:,1) = [0 0]';
    
  %Initialize the correct conditions.  
if strcmp(estimator,'kalman')
        x_hat_plus = zeros(4,N_pts);
        x_hat_plus(:,1) = [0 0 0 0]';
        P_plus(:,:,1) = zeros(4,4); 
elseif strcmp(estimator,'bry')
    x(:,1) = [0 0 0 0]';
    x_hat(:,1) = [0 0 0 0]';

    sigma_update = zeros(4,4,N_pts);
    lambda = zeros(4,4,N_pts);
    u_error = zeros(2,N_pts);
end

for k=2:N_pts
   x_nominal(:,k) = A*x_nominal(:,k-1) + B*u(:,k-1);
   %trajectory control
    u(:,k) = -K_ss*(x_nominal(:,k) - r(:,k)) + G*v(:,k) - R\B'*s(:,k);
    

   %     %state estimation
    switch estimator
        case 'kalman'
            [x_hat_plus(:,k), P_plus(:,:,k), ] = kalman(A, B, C, Q, R, u(:,k), x_nominal(:,k), P_plus(:,:,k-1), x_hat_plus(:,k-1), w(:,k));
        case 'bry'
            u_error(:,k) = -K_ss*x_hat(:,k-1) + G*v(:,k-1);
           [x_hat(:,k), sigma_update(:,:,k), lambda(:,:,k)] =  bry_estimation(A, B, C,Q, R, K_ss, x_hat(:,k-1), u_error(:,k-1), sigma_update(:,:,k-1), lambda(:,:, k-1), H, w(:,k));
            x(:,k) = (x_nominal(:,k) + x_hat(:,k));
        case 'sigma_points'

        case 'particle_filter'

    end
end

uncert = sqrt(sigma_update(1:2,1:2,end) + lambda(1:2,1:2,end));
%Plotting results
% if strcmp(estimator,'kalman')
%     subplot(2,2,1)
%     plot(t, r, 'k:', t, x_hat_plus)
%      str = strcat( 'Results from',estimator, ' estimation');
%     title(str)
%     subplot(2,2,2)
%     plot(t,u)
%     hold on
%     plot([0;15],[0;0], 'k:')
%     subplot(2,2,3)
%     
%     subplot(2,2,4)
%     plot(x_hat_plus(1,:), x_hat_plus(2,:))
%     title('Position of robot')
%     hold on
%     for i = 1:100:N
%         plot_gaussian_ellipsoid([x_hat_plus(1, i), x_hat_plus(2,i)], sqrt(P_plus(1:2,1:2,i)), 1, 25); % mean, std dev, stdwidth (2~= .86), n points. 
%     end
%     legend('estimated position', 'error ellipse', 'Location', 'NorthWest')
% elseif strcmp(estimator, 'bry')
% figure(2)
%     subplot(2,2,1)
%     plot(t, x)
%     ylabel('x, r')
%     hold on
%     plot(t,r,'k:')
%         str = strcat( 'Results from',estimator, ' estimation');
%     title(str)
%     subplot(2,2,2)
%     plot(t,u)
%     ylabel('u')
%     hold on
%     plot([0;15],[0;0], 'k:')
%     subplot(2,2,3)
%     hold on
% %     plot(t, x_hat)
%     ylabel('x_{hat}')
%     title('x error quantity')
%     subplot(2,2,4)
%     plot(x(1,:), x(2,:))
%     title('Position of robot')
%     hold on
%     for i = 1:100:N_pts
%         plot_gaussian_ellipsoid([x(1, i), x(2,i)], sqrt(sigma_update(1:2,1:2,i) + lambda(1:2,1:2,i)), 1, 25); % mean, std dev, stdwidth (2~= .86), n points. 
% %         plot_gaussian_ellipsoid([x(1, i), x(2,i)], sqrt(sigma_update(1:2,1:2,i)) , 2, 25); % mean, std dev, stdwidth (2~= .86), n points. 
%     plot(x(1,i), x(2,i), 'rx')
%     end
%     legend('estimated position', 'error ellipse', 'Location', 'NorthWest')
% end

end


function [x_hat_plus, P_plus] = kalman(A, B, C, Q, R, u, x_nominal, P_plus, x_hat_plus, w)
  %Regular Kalman Filter.
    x_hat_minus = A*x_hat_plus + B*u;
    P_minus = A*P_plus*A' + Q;
    z = C*x_nominal+ w;

    K = P_minus*C'/(C*P_minus*C' + R);
    x_hat_plus = x_hat_minus + K*(z - C*x_hat_minus);
    P_plus = inv(P_minus);
end


function [x_hat, sigma_update, lambda] =  bry_estimation(A, B, C, Q, R, K_ss, x_hat, u_error, sigma_update, lambda,H, w)
                            %     %prediction step
    x_pred = A*x_hat + B*u_error;
    sigma_pred = A*sigma_update*A' + Q;
    z_err = C*x_pred + H*w;  %Take a measurement, plus some noise.

%     %update step
    S = C*sigma_pred*C' + R;
    L = sigma_pred*C'/S;
    x_hat = x_pred + L*(z_err- C*x_pred);
    sigma_update = sigma_pred - L*C*sigma_pred;

    lambda = (A-B*K_ss)*lambda*(A-B*K_ss)'+ L*C*sigma_pred;
end


