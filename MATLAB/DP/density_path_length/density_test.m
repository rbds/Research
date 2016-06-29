clear


env = 10000; % size of environment (total area).
N = [100, 250, 500, 1000, 1500, 2000, 2500, 3000, 4000, 5000];  
% N = 250; %number of circles
ni = 10;

path_lengths = zeros(length(N), ni);
num_legs = zeros(length(N),ni);
for kk = 1:ni
    
    for ii = 1:length(N)

    centers= (env*2-1).*rand(2,N(ii)) + 0;

    % put obstacles in environment
    course = [0 0 sqrt(env) sqrt(env)];

    % Draw Course    
    obst = draw_obstacles(N(ii), centers, course);

    % define start and end points
    p_start = [1;2];
    p_goal = [99; 99];
    % run RRT 1000 iterations?

    % Parameters 
    param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
    param.thresh = 12;           %Bias towards the goal starting at this distance
    param.maxiters =  250;      %Cap on iterations to run RRT
    param.RRTstarrad = 25;      %Maximum length of lines redrawn by RRT*
    param.goalbias = .95;        %Probability of checking the goal as p_new
    param.maxpathlength = 100;    %Maximum length of any path segment.
    param.speed = 1;
    param.drain = .05;
    %param.smoothiters = 150;    %Number of iterations for smoothing algorithm
    goal.r = .5;            %radius of goal
    robot.r = 0.75;
    robot.t = 0;
    cost = 0;
    failure = 0;
    robot.p = p_start;


    circle(p_start(1),p_start(2),goal.r,'g');               %draw the location of x_start and x_goal
    circle(p_goal(1),p_goal(2),goal.r,'g');

    [P, T] = PathPlan(robot,obst,param,p_start,p_goal, course);
    %
    %output path length.
    for jj = 1:length(P) - 1
        cost = cost + norm(P(:,jj) - P(:,jj+1));
    end

    path_lengths(ii, kk) = cost;
    num_legs(ii,kk) = length(P);
    cost

    end

end
mean(path_lengths,1)