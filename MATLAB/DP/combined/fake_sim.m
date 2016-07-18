clear
close all

% env = 'pipeline';
env = 'field';

%create obstacles
[ costs, P_tr, obst, n_rows, n_cols ] = add_obstacles(env );
% M(1) = getframe;
%build adjacency matrix
V = n_rows*n_cols; %total number of nodes
% N = V^2;
i_vals = [];
j_vals = [];
dist = [];
if strcmp(env, 'pipeline')
    targets = [1, 11; 3,12; 5,11; 10,11; 14,9; 20,9; 25,9; 30,9; 36,4 ; 38,8; 40,8; 45, 7; 50,7];
else %field
    targets = [2,2; 48, 4; 45, 45; 20, 32; 2, 48; 2,2];
end
tar = 1;

P_tr_thresh = .850;

tic
for i = 1:V    %for each row in adjacency matrix
   if (mod(i,n_cols) >0) %if it isn't on the right edge of grid 
    j_vals(end+1) = i; %add node to right
    i_vals(end+1) = i+1;
    dist(end+1) = 1;
   end

   if (mod(i, n_cols) ~=1) %if node isn't on the left edge of grid
    j_vals(end+1) = i; %add node to left
    i_vals(end+1) = i-1;
    dist(end+1) = 1;
   end
   
   if (i> n_cols) %if vertex isn't on top of grid
    j_vals(end+1) = i ;   %add next node up
    i_vals(end+1) = i - n_cols;
    dist(end+1) = 1;
       if (mod(i,n_cols) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1 - n_cols;
        dist(end+1) = sqrt(2);
       end
% 
       if (mod(i, n_cols) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1 -n_cols;
        dist(end+1) = sqrt(2);
       end
   end
   
   if (i<=(V- n_cols))   %if vertex isn't on bottom of grid
    j_vals(end+1) = i ;  %add next node down
    i_vals(end+1) = i+n_cols ;
    dist(end+1) = 1;
       if (mod(i,n_cols) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1+ n_cols;
        dist(end+1) = sqrt(2);
       end
% 
       if (mod(i, n_cols) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1+n_cols;
        dist(end+1) = sqrt(2);
       end
   end   
end

% %populate coordinates for plotting
coords = [];
for i = 1:V
   col = mod(i,n_cols);
   row = ceil((i)/n_cols);
      if (col==0) col = n_cols; end 
   coords(end+1,:) = [row, col]; 
end

%create adjacency matrix
vals = zeros(size(i_vals));
for i=1:length(i_vals)
    vals(i) = mean([costs(i_vals(i)), costs(j_vals(i))])*dist(i);
end
adj= sparse(i_vals, j_vals, vals); %one section of the adjacency matrix

% M(end+1) = getframe;

% [adj_i, adj_j, adj_v] = find(adj); %access rows and columns of adjacency matrix.

P_tr(V) = 1;
d{V,1} = []; %create d vector to store cost, P_tr, parents for each entry in adj.
bp = [];

for jj= 1:length(targets)-1
    
    clear d
    d{V,1} = [];
    
    start_node = (targets(tar,1)-1)*n_cols + targets(tar,2);
    tar = tar+1;
    target_node = (targets(tar,1)-1)*n_cols + targets(tar,2);  
    [d, best_path] =  find_path(start_node, target_node, env, V, d, adj, coords, P_tr, P_tr_thresh);

    hold on
    for i=1:length(best_path)-1  %plot path
    %     plot(coords(best_path(i),1), coords(best_path(i),2), 'r*')
      h0 =   plot([coords(best_path(i),1), coords(best_path(i+1),1)],[coords(best_path(i),2), coords(best_path(i+1),2)], 'r-', 'LineWidth', 4);    
%     M(end+1) = getframe;
    end
    axis off
    
    bp = cat(2, bp, best_path);
end
figure
plot_maps(V, coords, costs, P_tr, env);

figure(1)

clear dist i_vals j_vals vals

p_start =  coords(bp(1),:)'; 
course = [1 1 15 15];
s = [];
map = [];
param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
param.thresh = 12;           %Bias towards the goal starting at this distance
param.maxiters = 100;      %Cap on iterations to run RRT
param.RRTstarrad = 15;      %Maximum length of lines redrawn by RRT*
param.goalbias = .95;        %Probability of checking the goal as p_new
param.maxpathlength = 20;    %Maximum length of any path segment.
param.sensor_range = 4;
param.gas_tank = 45;
goal.r = .5;            %radius of goal
robot.r = 0.4;
robot.t = 0;

robot.p = p_start;               %set robot position to x_start
robot.v = [0; 0; 0];


% circle(p_start(1),p_start(1),goal.r,'g');               %draw the location of x_start and x_goal
% M(end+1) = getframe;

ka = 1.5;        %attractive gain
kr = 3;        %repulsive gain

dt = .02;        %time step size (seconds)

%%%%%%%%%%%%%%while robot position != goal:
h = draw_robot(robot);
% M(end+1) = getframe;
F = [0 0];
dif = 0;
t_dist = 0;


clear adj d dist i_vals j_vals vals
for ii = 1:(length(bp))
    p_goal = coords(bp(ii), :)';
    while norm(robot.p - p_goal) > robot.r + .2
        %%%%%%%%%Define robot position
    %     robot.x = [robot.p; robot.v];
        map = [];
        %%%%%%%%%%%do a sensor sweep
       [ map, s ] = sensor( robot, obst, map, s, param.sensor_range, course);
%         [ map, s, M ] = sensor( robot, obst, map, s, param.sensor_range, course, M);

        %%%%%%%%%%%Find potential function
            %attractive potential
             dU_a = (robot.p - p_goal)/norm(robot.p - p_goal)*ka;


            %repulsive potential
            dU_r = zeros(size(map,1),2);
            for ii=1:size(map,1)
                for jj = size(map(ii).p,1)
                    d_to_obst = norm(-map(ii).p(jj,:) + robot.p');
                    del_r = (-map(ii).p(jj,:) + robot.p')/norm(-map(ii).p(jj,:) + robot.p');
                    dU_r(ii,:) = kr*(1/param.sensor_range - 1/d_to_obst)*1/d_to_obst^2*del_r;
                end
            end
            F_old = F;
            F = sum([-dU_a'; -dU_r],1);
        %%%%%%%%%%%% Move robot for one timestep
            plot(robot.p(1), robot.p(2), 'gx')
%             M(end+1) = getframe;
            old_p = robot.p;
            xdd = F-F_old; %previous desired velocity.    
            robot = state_int(robot, F, dt, xdd);
%             robot = si(robot, F, dt);
            plot([old_p(1), robot.p(1)],[old_p(2), robot.p(2)],'g', 'LineWidth', 3)
%             M(end+1) = getframe;
%             dif = norm(old_p - robot.p(1:2));
%             if dif < .05
%                 disp('break')
%                 break
%             end   

            set(h, 'Visible', 'off')
            h = draw_robot(robot);
            set(h, 'Visible', 'on')
            drawnow
%             M(end+1) = getframe;
            
    end
end






% 
% run('make_a_movie.m')
