%state = [x y x_dot y_dot]'
%input = force in x and y direction
clf
clear


m = 1;
cx = 2;
cy = 3;
kx = 0;
ky = 0;

A = [0 0 1 0; 0 0 0 1; kx/m 0 cx/m 0; 0 ky/m 0 cy/m];
% A = [ 0 0 1 1; 0 0 1 1; 0 0 0 0; 0 0 0 0];
B =[0 0 1/m 0; 0 0 0 1/m]';
% B= [0 0 1 1; 0 0 1 1]';
C = [1 0 0 0; 0 1 0 0];
Q = 2*eye(4);
% Q = [1 0 0 0; 0 1 0 0; 0 0 0 0; 0 0 0 0];
R = 0.25*[1 0; 0 1];
% R2 = .05;

kalman_q = .001*eye(4);
kalman_r = .1*eye(2);

%Find Kalman and LQR Gain.
[K_ss, P, E] = lqr(A,B,Q,R); %K is lqr gain, P is solution to Ricca1ti eqn, E is eig(A-BK) of system
[LT, PK, EK] = lqr(A', C', kalman_q, kalman_r); %Kalman Filter
L_ss= LT';
L1 = [4 0; 0 6; 9 0; 0 19];
x(:,1) = [1 1 0 0]';
x_hat(:,1) = [1 1 0 0]';
N = 5000;
tf = 10;
t = linspace(0, tf, N);
dt = t(2) - t(1);
r(:,1) = linspace(1, 8, N);
r(:,2) = linspace(1, 7, N);
r(:,3) = -(r(1,1) - r(2,1))/dt;
r(:,4) = -(r(1,2) - r(2,2))/dt;
u = zeros(2, N);
s = zeros(4,N);
r = r';

% s(:,N) =  -Q*r(:,N);
P = zeros(4,4,N);
P_kalman = zeros(4,4,N);
for k= N:-1:2
    P_dot= - P(:,:,k)*A - A'*P(:,:,k) + P(:,:,k)*B*inv(R)*B'*P(:,:,k)- Q;
    s_dot= (P(:,:,k)*B*inv(R)*B'- A')*s(:,k) + Q*r(:,k);
    P(:,:,k-1)= P(:,:,k) - P_dot*dt;
    s(:,k-1)= s(:,k) - s_dot*dt;
end


w = .01*randn(2, N);
v = .2*randn(2, N);
H = eye(2);
G = [1 0; 0 1; 0 0; 0 0];

for k=1:N-1    
      P_dot_kalman = -P_kalman(:,:,k)*A' - A*P(:,:,k) + P_kalman(:,:,k)*C'*inv(kalman_r)*C*P_kalman(:,:,k) - kalman_q;
%       s_dot_kalman = ((P_kalman(:,:,k)*C'*inv(kalman_r))' - A)*s_kalman(:,k);
      P_kalman(:,:,k+1) = P_kalman(:,:,k) + P_dot_kalman*dt;
%       s_kalman(:,:,k+1) = s_kalman + s_dot_kalman*dt;
   K(:,:,k) =  R\B'*P(:,:,k);
   L(:,:,k) = P_kalman(:,:,k)*C'*inv(kalman_r);
   %predicted values
   z(:,k) = C*x(:,k)+ w(:,k); %output from sensors, plus some sensing noise.
   y(:,k) = C*x(:,k);
   u(:,k)= - K(:,:,k)*x_hat(:,k) - R\B'*s(:,k) + H*v(:,k);
   x_dot_hat(:, k+1) = A*x_hat(:,k) + B*u(:,k) - L_ss*(y(:,k) - C*x_hat(:,k));   %predicted value for x_dot
   x_hat(:,k+1) = x_hat(:,k) + x_dot_hat(:,k)*dt;
   
   %actual values
   x_dot(:,k+1)= A*x(:,k)+ B*u(:,k);% %actual value for x_dot
   x(:,k+1)= x(:,k) + x_dot(:,k)*dt;
end

figure(1)
clf
subplot(2,2,1)
plot(t,x, 'r:')
ylabel('x, r')
hold on
plot(t,r,'k')
subplot(2,2,2)
plot(t,u)
ylabel('u')
subplot(2,2,3)
plot(t(1:end-1), z, t(1:end-1), y)
ylabel('z')
hold on 
subplot(2,2,4)
plot(t, x_hat)
hold on
% plot(t, x_hat(3,:))
ylabel('x_{hat}, r')
plot(t,r, ':')

figure(2)
plot(t, x_dot, t, x_dot_hat)
 legend('x dots', 'x hat dots')
