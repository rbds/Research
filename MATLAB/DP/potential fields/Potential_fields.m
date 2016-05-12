%Potential Field Method

%%%%%%%%%Define a map with obstacles, start, and target locations
% close all
clf
figure(1)
hold on
clear


%create obstacles by code         
course = [0 0 45 33];
numberOfObstacles = 5;
s = [];
map = [];
% Draw Course    
obst = draw_obstacles(numberOfObstacles, course);

% Set Start and Goal locations
p_start = [5;28];
p_goal = [41; 6];

% Parameters 
param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
param.thresh = 12;           %Bias towards the goal starting at this distance
param.maxiters = 100;      %Cap on iterations to run RRT
param.RRTstarrad = 15;      %Maximum length of lines redrawn by RRT*
param.goalbias = .95;        %Probability of checking the goal as p_new
param.maxpathlength = 20;    %Maximum length of any path segment.
param.sensor_range = 6;
%param.smoothiters = 150;    %Number of iterations for smoothing algorithm
goal.r = .5;            %radius of goal
robot.r = 0.75;
robot.t = 0;

robot.p = p_start;               %set robot position to x_start

circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');

ka = .2;        %attractive gain
kr = .5;        %repulsive gain
q_thresh = 10;  %max distance for obstacle to produce a virtual force

dt = .2;        %time step size (seconds)


%%%%%%%%%%%%%%while robot position != goal:
h = draw_robot(robot);
while norm(robot.p - p_goal) > robot.r+goal.r
    %%%%%%%%%Define robot position
    map = [];
    %%%%%%%%%%%do a sensor sweep
    [ map, s ] = sensor( robot, obst, map, s, param.sensor_range, course);

    %%%%%%%%%%%Find potential function
        %attractive potential
        dU_a = ka*(robot.p - p_goal);
%         plot([robot.p(1), robot.p(1)-dU_a(1)], [robot.p(2), robot.p(2)-dU_a(2)], 'g' )
   
        %repulsive potential
        dU_r = zeros(length(map),2);
        for i=1:length(map)
            d_to_obst = norm(-map(i).p + robot.p);
            del_r = (-map(i).p + robot.p)/norm(-map(i).p + robot.p);
            dU_r(i,:) = kr*(1/q_thresh - 1/d_to_obst)*1/d_to_obst^2*del_r;
%             plot([robot.p(1), robot.p(1)+dU_r(i,1)], [robot.p(2), robot.p(2)+dU_r(i,2)], 'r' )
        end
        
    %%%%%%%%%%%Find gradient
        F = sum([-dU_a';-dU_r],1);

    %%%%%%%%%%%% Move robot for one timestep
        plot(robot.p(1), robot.p(2), 'bx')
        old_p = robot.p;
        robot = state_int(robot, F, dt);
    
        plot([old_p(1), robot.p(1)],[old_p(2), robot.p(2)],'g', 'LineWidth', 3)
        set(h, 'Visible', 'off')
        h = draw_robot(robot);
        set(h, 'Visible', 'on')
        drawnow
end