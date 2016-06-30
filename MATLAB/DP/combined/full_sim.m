clear
close all

env = 'sample';
% env = 'pipeline';

%create obstacles
[ costs, P_tr, obst, n_rows, n_cols ] = add_obstacles(env );
M(1) = getframe;
%build adjacency matrix
V = n_rows*n_cols; %total number of nodes
% N = V^2;
i_vals = [];
j_vals = [];
dist = [];

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

if strcmp(env, 'sample')
    gplot(adj, coords, '*-') %plot graph
    axis([0 16 0 16])
end
M(end+1) = getframe;

[adj_i, adj_j, adj_v] = find(adj); %access rows and columns of adjacency matrix.

P_tr(V) = 1;
d{V,1} = []; %create d vector to store cost, P_tr, parents for each entry in adj.

d{V} = [0 1 i*V]; 
    
for i=V:-1:1 %counting back from last populated column in adjacency matrix,
% for i=rows_to_do
    col = i;
        V_col = mod(col, V);
        if (V_col==0) V_col = V; end 
    conns = find(adj(:,col)~=0);    %find entries in column col
    
    if mod(i,1000)==0
       disp('on column: ')
       disp(i)
    end
    
    for j=1:length(conns)   %for each connection,
        if isempty(d{conns(j)})
            options = [];
        else
            options = d{conns(j),1}; %pull current pareto front
        end
        row = conns(j);             %row of current connection
        V_row = mod(row, V);
        if (V_row==0) V_row = V; end 

        new_options = [d{i,1}(:,1) + adj(row,col), d{i,1}(:,2)*P_tr(V_row), repmat(i, size(d{i,1}(:,1)))];   %list possible new paths
        
    options = [options; new_options];
    [front, inds] = prto(options);
       
    if isempty(front) 
        d{conns(j),1} = options;

    else
        d{conns(j),1} = front;
    end
  
    end
end


time = toc;
hold on
bad_paths = 0;
for i=1:length(d)
    bad_paths = find(d{i}(:,2)<P_tr_thresh);
    d{i}(bad_paths, 1) = d{i}(bad_paths,1) + 1e6;
end
best_path = extract_best_path(d, 1, V);
cost = min(d{1}(:,1))
Ptr = prod(P_tr(best_path))
% Ptr = d{1}(1,2)
time

hold on
for i=1:length(best_path)-1  %plot path
%     plot(coords(best_path(i),1), coords(best_path(i),2), 'r*')
  h0 =   plot([coords(best_path(i),1), coords(best_path(i+1),1)],[coords(best_path(i),2), coords(best_path(i+1),2)], 'r-', 'LineWidth', 4);    
M(end+1) = getframe;
end
axis off
figure

[c, p] = plot_paths( d, best_path, costs, P_tr, coords);

figure
plot_maps(V, coords, costs, P_tr);

figure(1)

p_start = [1;1]; 
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
goal.r = .5;            %radius of goal
robot.r = 0.4;
robot.t = 0;

robot.p = p_start;               %set robot position to x_start
robot.v = [0; 0; 0];


circle(p_start(1),p_start(1),goal.r,'g');               %draw the location of x_start and x_goal
M(end+1) = getframe;

ka = 2;        %attractive gain
kr = 4;        %repulsive gain

dt = .05;        %time step size (seconds)

%%%%%%%%%%%%%%while robot position != goal:
h = draw_robot(robot);
M(end+1) = getframe;
F = [0 0];
for ii = 1:(length(best_path))

    
    p_goal = coords(best_path(ii), :)';
    while norm(robot.p - p_goal) > robot.r
        %%%%%%%%%Define robot position
    %     robot.x = [robot.p; robot.v];
        map = [];
        %%%%%%%%%%%do a sensor sweep
        [ map, s, M ] = sensor( robot, obst, map, s, param.sensor_range, course, M);

        %%%%%%%%%%%Find potential function
            %attractive potential
    %         dU_a = ka*(robot.p - p_goal)*sqrt(norm(robot.p - p_goal))/norm(robot.p - p_goal);
    %         dU_a = ka*(robot.p -p_goal);
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
            Fold = F;
            F = sum([-dU_a'; -dU_r],1);

        %%%%%%%%%%%% Move robot for one timestep
            plot(robot.p(1), robot.p(2), 'gx')
            M(end+1) = getframe;
            old_p = robot.p;
            xdd = F-Fold; %previous desired velocity.    
            robot = state_int(robot, F, dt, xdd);
            plot([old_p(1), robot.p(1)],[old_p(2), robot.p(2)],'g', 'LineWidth', 3)
            M(end+1) = getframe;
            
    %         robot.p = robot.p + dt*F';

            set(h, 'Visible', 'off')
            h = draw_robot(robot);
            set(h, 'Visible', 'on')
            drawnow
            M(end+1) = getframe;
    end

end
