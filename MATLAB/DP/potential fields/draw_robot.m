function [ h ] = draw_robot( rob )
%draw_robot Plots the robot, returns plot handle
%   Detailed explanation goes here

th = 0:pi/50:2*pi;
xunit = rob.r * cos(th) + rob.p(1);
yunit = rob.r * sin(th) + rob.p(2);

center = rob.p;
front = [0 0]';
front(1) = center(1) + rob.r*cos(rob.t);
front(2) = center(2) + rob.r*sin(rob.t);

h = plot([center(1);front(1)],[center(2);front(2)], xunit, yunit,'c','LineWidth',2);
drawnow;


end 

