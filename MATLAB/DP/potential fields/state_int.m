function [ robot ] = state_int( robot, F, dt )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% robot.p = robot.p + dt*F';
x2_d = [F, 0]';
x1_d = [robot.p; robot.t] + x2_d*dt;

x1 = x1_d - [robot.p(1); robot.p(2); robot.t];
x2 = x2_d - robot.v;


x1_dot = x2;
x2_dot = x_diff(x1, x2);

x1_new = x1 + dt*x1_dot;
x2_new = x2 + dt*x2_dot;

robot.p = x1_new(1:2);
robot.t = x1_new(3);

robot.v = x2_new;
end

function [x_ddot] = x_diff(x1, x2)
m = 116;
I = 20;
r = .2;
t = .63/2;
a = .37;
b = .55;
g = 9.81;

fr = .1;
mu = .895;

Rx = fr*m*g/2*(sign(x2(1)-t*x2(3)));
Fy = mu*m*g/(a+b)*(b*sign(x2(2)+ a*x2(3)) + a*sign(x2(2)-b*x2(3)));
Mr = mu*a*b*m*g/(a+b)*(sign(x2(2)+ a*x2(3))- sign(x2(2)-b*x2(3)));

M = [m 0 0; 0 m 0; 0 0 I];
c = [Rx*cos(x1(3)) - Fy*sin(x1(3)); Rx*sin(x1(3)) + Fy*cos(x1(3)); Mr];
E = [cos(x1(3))/r, cos(x1(3))/r; sin(x1(3))/r, sin(x1(3))/r; t/r, -t/r];
inv_E = inv(E'*E)*E';


rho = abs(x2) + inv(M)*c;
beta = 2;
eps = .01;

s = x1(1:2) + x2(1:2);
u = -inv_E*M*(rho + beta).*sat(s/eps);

x_ddot =  M\(E*u - c);
end

function y=sat(x)

if abs(x) < 1
    y=x;
else
    y=sign(x);
end

end