% clear
% clf
% 
% env = 10000; % size of environment (total area).
% N = [50, 100,  150, 200, 250, 300, 350, 400, 450, 500, 600, 700, 800, 900, 1000]; %, 2500, 3000, 4000, 5000];
% n_fills = length(N);
% % N = 100;
% ni = 25;
% 
% path_lengths = zeros(n_fills, ni);
% num_legs = zeros(n_fills,ni);
% for kk = 1:ni
%     
%     for ii = 1:n_fills
% % ii=1;
%     clf
%     centers= (sqrt(env)-1).*rand(2,N(ii)) + 0;
% 
%     % put obstacles in environment
%     course = [0 0 sqrt(env) sqrt(env)];
% 
%     % Draw Course    
%     obst = draw_obstacles(N(ii), centers, course);
% 
%     % define start and end points
%     p_start = [1;2];
%     p_goal = [99; 99];
%     % run RRT 1000 iterations?
% 
%     % Parameters 
%     param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
%     param.thresh = 12;           %Bias towards the goal starting at this distance
%     param.maxiters =  500;      %Cap on iterations to run RRT
%     param.RRTstarrad = 25;      %Maximum length of lines redrawn by RRT*
%     param.goalbias = .95;        %Probability of checking the goal as p_new
%     param.maxpathlength = 100;    %Maximum length of any path segment.
%     param.speed = 1;
%     param.drain = .05;
%     %param.smoothiters = 150;    %Number of iterations for smoothing algorithm
%     goal.r = .5;            %radius of goal
%     robot.r = 0.75;
%     robot.t = 0;
%     cost = 0;
%     failure = 0;
%     robot.p = p_start;
% 
% 
%     circle(p_start(1),p_start(2),goal.r,'g');               %draw the location of x_start and x_goal
%     circle(p_goal(1),p_goal(2),goal.r,'g');
% 
%     [P, T] = PathPlan(robot,obst,param,p_start,p_goal, course);
%     %
%     %output path length.
%     if size(P,2)> 2
%         for jj = 1:length(P) - 1
%             cost = cost + norm(P(:,jj) - P(:,jj+1));
%         end
%     else
%         cost = 0;
%     end
%     
% %     %plot path
% %     for i = 1:length(P) - 1 %Draw Path
% %       if (length(P) ~= 1)
% %       plot([P(1,i);P(1, i+1)],[P(2,i);P(2,i+1)], 'g', 'LineWidth', 3) 
% %       end
% %     end
% %     figure(gcf)
% 
%     path_lengths(ii, kk) = cost;
%     num_legs(ii,kk) = length(P);
%     cost
% 
%     end
% 
% end
% % mean(path_lengths,1)
% use = path_lengths>0;
% p = zeros(n_fills,1);
% 
% for ii = 1:n_fills
%    p(ii) = mean(path_lengths(ii,use(ii,:)));
% end

figure
plot(N./env, p/norm(p_goal-p_start), 'b*', 'LineWidth', 5)
% f = fit(N(1:end-1)'./env, p(1:end-1)/norm(p_goal-p_start),'exp2');
hold on
plot(f, 'r')
title('Path Length vs. Obstacle Density')
xlabel('Obstacle Density (%)')
ylabel('Ratio of path length to straight line')
legend('numerical simulation', 'exponential curve fit')
set(gca,'fontsize', 18)

