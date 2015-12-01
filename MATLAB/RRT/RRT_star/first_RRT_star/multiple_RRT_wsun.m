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
   p_sun = [40; 26];
%         th = 0:pi/50:2*pi;
%         xunit = .5 * cos(th) + p_sun(1);
%         yunit = .5 * sin(th) + p_sun(2);
%         orange = [230 218 57]./256;
%         plot(xunit, yunit,'LineWidth',3, 'Color', orange);
%         drawnow
% Parameters 
param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
param.thresh = 12;           %Bias towards the goal starting at this distance
param.maxiters =  250;      %Cap on iterations to run RRT
param.RRTstarrad = 15;      %Maximum length of lines redrawn by RRT*
param.goalbias = .95;        %Probability of checking the goal as p_new
param.maxpathlength = 100;    %Maximum length of any path segment.
param.speed = 1;
param.drain = .04;
%param.smoothiters = 150;    %Number of iterations for smoothing algorithm
goal.r = .5;            %radius of goal
robot.r = 0.75;
robot.t = 0;
failure = 0;
robot.p = p_start;               %set robot position to x_start

circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');

% Plan the path
% M(1) = getframe;
cost =0;

[P, T] = PathPlan(robot,obst,param,p_start,p_goal, course);
for i = 1:length(P) - 1
    cost = cost + norm(P(:,i) - P(:,i+1));
end
SOC = 1 - cost*param.drain;
if SOC < 0
tic;
tstart = tic;

  param.maxiters = round(param.maxiters/2);
  figure
  hold on
  [P, T] = PathPlan(robot,obst,param,p_start, p_sun, course);
  cost = 0;
    for i = 1:length(P) - 1
         cost = cost + norm(P(:,i) - P(:,i+1));
    end
    SOC = 1 - cost*param.drain;
    if SOC < 0
       failure = 1;
    else
        SOC = 1;
    end
  figure
  hold on
  [P1, T1] = PathPlan(robot,obst,param,p_sun, p_goal, course); 
  
    for i = 1:length(P1) - 1
         cost = cost + norm(P1(:,i) - P1(:,i+1));
    end
    SOC = 1 - cost*param.drain;
    if SOC < 0
      failure = 1;
    end
  P = cat(2, P, P1);
end
figure
hold on
trees = exist('P1','var');
if trees ==1
    for i = 1: length(T)
     plot([T(i).p(1); T(T(i).iPrev).p(1)], [T(i).p(2);T(T(i).iPrev).p(2)], 'LineWidth', 2) 
    end
    for i = 1: length(T1)
     plot([T1(i).p(1); T1(T1(i).iPrev).p(1)], [T1(i).p(2);T1(T1(i).iPrev).p(2)], 'b', 'LineWidth', 2) 
    end
else
    for i = 1: length(T)
     plot([T(i).p(1); T(T(i).iPrev).p(1)], [T(i).p(2);T(T(i).iPrev).p(2)], 'LineWidth', 2) 
    end
end

obst = draw_obstacles(numberOfObstacles, course);
circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');
        th = 0:pi/50:2*pi;
        xunit = .5 * cos(th) + p_sun(1);
        yunit = .5 * sin(th) + p_sun(2);
        orange = [230 218 57]./256;
plot(xunit, yunit,'LineWidth',3, 'Color', orange);
        drawnow  
        
for i = 1:length(P) - 1 %Draw Path
      if (length(P) ~= 1)
      plot([P(1,i);P(1, i+1)],[P(2,i);P(2,i+1)], 'g', 'LineWidth', 3) 
      end
end
telapsed = toc(tstart)
cost
%add M as a returned argument to return movie
   
%          M(end +1) = getframe;
    
   
    %P is successful path
    %T is full tree of vertices and parents
    %iterations is number of iterations
    %T(end).cost will give the cost of the successful path

