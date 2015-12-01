% Current RRT Script
%Original: University of Illinois
%Last Modified:  by Randy Schur

% %TO DO: 

% %ERROR CODES:
close all
figure
hold on
clear


%create obstacles by code         
course = [0 0 45 33];
numberOfObstacles = 5;

% Draw Course    
obst = draw_obstacles(numberOfObstacles, course);

% Set Start and Goal locations
p_start = [5;28];
p_goal = [41; 6];
%        %draw the sun.
%    p_sun = [40; 26];
p_sun = [24;15];
        th = 0:pi/50:2*pi;
        xunit = .5 * cos(th) + p_sun(1);
        yunit = .5 * sin(th) + p_sun(2);
        orange = [230 218 57]./256;
        plot(xunit, yunit,'LineWidth',3, 'Color', orange);
        drawnow
% Parameters 
param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
param.thresh = 12;           %Bias towards the goal starting at this distance
param.maxiters =  250;      %Cap on iterations to run RRT
param.RRTstarrad = 15;      %Maximum length of lines redrawn by RRT*
param.goalbias = .95;        %Probability of checking the goal as p_new
param.sunbias = .90;
param.maxpathlength = 100;    %Maximum length of any path segment.
param.speed = 1;
param.drain = .03;
param.minSOC = .01;
%param.smoothiters = 150;    %Number of iterations for smoothing algorithm
goal.r = .5;            %radius of goal
robot.r = 0.75;
robot.t = 0;
cost = 0;
pathSOC = 1;
robot.p = p_start;               %set robot position to x_start

circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');

% Plan the path
% M(1) = getframe;

[P, T] = PathPlan(robot,obst,param,p_start,p_goal, p_sun, course);
figure
hold on

for i = 1: length(T)
   plot([T(i).p(1); T(T(i).iPrev).p(1)], [T(i).p(2);T(T(i).iPrev).p(2)], 'LineWidth', 2) 
end
obst = draw_obstacles(numberOfObstacles, course);
circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');
        th = 0:pi/50:2*pi;
        xunit = .5 * cos(th) + p_sun(1);
        yunit = .5 * sin(th) + p_sun(2);
        orange = [230 218 57]./256;
plot(xunit, yunit,'LineWidth',3, 'Color', orange); 

for i = 1:length(P) - 1 %Draw Path so far.
      if (length(P) ~= 1)
      plot([P(1,i);P(1, i+1)],[P(2,i);P(2,i+1)], 'g', 'LineWidth', 3) 
      end
      cost = cost + norm(P(:,i) - P(:,i+1));
      pathSOC = pathSOC - param.drain*norm(P(:,i) - P(:, i+1));
      if norm(P(:,i)-p_sun) < goal.r
          pathSOC =1;
      end
end

%add M as a returned argument to return movie
   
%          M(end +1) = getframe;
    
   
    %P is successful path
    %T is full tree of vertices and parents
    %iterations is number of iterations
    %T(end).cost will give the cost of the successful path

