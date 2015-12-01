%Discrete LQR + Kalman Filter
function [] = estimation()

clear
figure(1)
% clf
% figure
%choose estimator type
% estimator = 'kalman';
estimator = 'bry';
%estimator = 'sigma_points';
% estimator = 'particle_filter';
% global A B C;


m = 10;
cx = 3;
cy = 5;
kx = 0;
ky = 0;

Q = .02*eye(4); %noise covariances.
R = .01*eye(2);

N = 1000;   %number of iterations to run.
tf = 15;


r(:,1) = linspace(0, 80, N);     %reference trajectory
r(:,2) = linspace(0, 70, N);
r(:,3) = zeros(1,N);
r(:,4) = zeros(1,N);
r = r';
r(1, N+1:2*N) = linspace(80, 200, N);
r(2, N+1:2*N) = linspace(70, 120, N);
r(3,N+1:2*N) = zeros(1,N);
r(4,N+1:2*N) = zeros(1,N);

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

Q_lqr = 10*[10 0 0 0; 0 10 0 0; 0 0 0 0; 0 0 0 0];       %lqr cost function matrices.
R_lqr = 0.1*eye(2);
[K_ss, ~, ~] = dlqr(A, B, Q_lqr, R_lqr); %K is lqr gain, P is solution to Riccatti eqn, E is eig(A-BK) of system
% [L_ss, ~, ~] = dlqr(A', C', Q, R);

w = .1*randn(2, N_pts);        %gaussian noise
v = .2*randn(2, N_pts);
G = eye(2);
H = eye(2);

% % for tracking controller
P = zeros(4,4,N_pts);
s = zeros(4,4,N_pts);
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
elseif strcmp(estimator, 'sigma_points')
        x_hat_plus = zeros(4,N_pts);
        x_hat_plus(:,1) = [0 0 0 0]';
        P_plus(:,:,1) = zeros(4,4);
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
            [x_hat_plus(:,k), P_plus(:,:,k), ] = sigma_points(A, B, C, Q, R, u(:,k), x_hat_plus(:,k-1), P_plus(:,:,k-1));
        case 'particle_filter'

    end
end

%Plotting results
if strcmp(estimator,'kalman')
    subplot(2,2,1)
    plot(t, r, 'k:', t, x_hat_plus)
     str = strcat( 'Results from',estimator, ' estimation');
    title(str)
    subplot(2,2,2)
    plot(t,u)
    hold on
    plot([0;15],[0;0], 'k:')
    subplot(2,2,3)
    
    subplot(2,2,4)
    plot(x_hat_plus(1,:), x_hat_plus(2,:))
    title('Position of robot')
    hold on
    for i = 1:100:N
        plot_gaussian_ellipsoid([x_hat_plus(1, i), x_hat_plus(2,i)], sqrt(P_plus(1:2,1:2,i)), 1, 25); % mean, std dev, stdwidth (2~= .86), n points. 
    end
    legend('estimated position', 'error ellipse', 'Location', 'NorthWest')
elseif strcmp(estimator, 'bry')
    subplot(2,2,1)
    plot(t, x)
    ylabel('x, r')
    hold on
    plot(t,r,'k:')
        str = strcat( 'Results from {}',estimator, ' estimation');
    title(str)
    subplot(2,2,2)
    plot(t,u)
    ylabel('u')
    hold on
    plot([0;15],[0;0], 'k:')
    subplot(2,2,3)
    hold on
    plot(t, x_hat)
    ylabel('x_{hat}')
    title('x error quantity')
    subplot(2,2,4)
    plot(x(1,:), x(2,:))
    title('Position of robot')
    hold on
    for i = 1:100:N_pts
        plot_gaussian_ellipsoid([x(1, i), x(2,i)], sqrt(sigma_update(1:2,1:2,i) + lambda(1:2,1:2,i)), 1, 25); % mean, std dev, stdwidth (2~= .86), n points. 
%         plot_gaussian_ellipsoid([x(1, i), x(2,i)], sqrt(sigma_update(1:2,1:2,i)) , 2, 25); % mean, std dev, stdwidth (2~= .86), n points. 
    plot(x(1,i), x(2,i), 'rx')
    end
    legend('estimated position', 'error ellipse', 'Location', 'NorthWest')
end

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

function [x_plus, P_plus] = sigma_points(A, B, C, Q, R, u, x_hat, P)
        n = 4;
        kap = 50;
        alpha = 0.5;
        beta = 2;
        lam = alpha^2*(n+kap) - n;
        wm(1) = lam/(n+lam);
        wv(1) = wm(1) + (1-alpha^2+beta^2);
       %choose sigma points
        sqrt_P=sqrtm((n+lam)*P);
        for i=2:2*n+1
            wm(i)=1/(2*(n+lam));
            wv(i)=wm(i);
        end
        sig_pts(1,:) = x_hat;
        for i=1:n
            sig_pts(2,:) = x_hat + sqrt_P(:,i);
        end
       %propogate for prediction step 
        for i=1:2*n+1
            x_sigma(i,:) = A*sig_pts(2,:)' + B*u;
        end
        %recover mean and variance: 
        x_minus=zeros(n,1);
        for i=1:length(wm)
            x_minus=x_minus+wm(i)*x_sigma(:,i);
        end
        P_minus=zeros(n,n);
        for i=1:length(wm)
            P_minus=P_minus+wv(i)*(x_sigma(:,i)-x_minus)*(x_sigma(:,i)-x_minus)';
        end
        P_minus = P_minus + Q;
  
        
        %Choose sigma points for correction step
         
         for i=2:2*n+1
            wm(i)=1/2/(n+lam);
            wv(i)=wm(i);
         end
         sig_pts(1,:) = x_hat;
        for i=1:n
            sig_pts(2,:) = x_hat + sqrt_P(:,i);
        end
        %propogate points (take measurement)
          for i=1:2*n+1
              z(i,:) = C*sig_pts(i,:)+ w;
          end
        %Recover mean and variance
        for i=1:length(wm)
            z_bar=z_bar+wm(i)*z(i,:);
        end
        P_z=zeros(n,n);
        for i=1:length(wm)
            P_z=P_z+wv(i)*(z(:,i)-z_bar)*(z(:,i)-z_bar)';
        end
        P_z = P_z + R;
        P_xz=zeros(n,2);
        for i=1:2*n+1
            P_xz=P_xz+wv(i)*(x_sigma(:,i)-xminus)*(z(:,i)-z_bar)';
        end
     	%correction
        K=P_xz*inv(P_z);
        x_plus=x_minus+K*(z - z_bar);
        P_plus=P_minus - K*P_z*K';


        
        
end
