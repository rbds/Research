clear
close all

% subplot(1,3,1)
ha = axes('units','normalized','position',[0 0 1.1 1.1]);

uistack(ha,'bottom');

I=imread('sample_map.png');
hi = imagesc(I);
colormap gray

set(ha,'handlevisibility','off','visible','off')

axes('position',[0 0 .99 .99])
axis off
axis equal

costs =[01 .1 .1 01 01 01 .6 .6 01 01 01 01 01 01 01;
        01 .1 .1 01 01 .3 .7 .6 01 01 01 01 .3 01 01;
        01 .1 .1 01 01 .8 .8 .8 .9 01 01 .3 .6 .5 .4;
        01 01 .1 .1 01 01 .9 .9 01 01 01 .3 .5 .5 .5;
        01 01 01 .1 .1 01 01 01 01 01 01 01 01 .3 01;
        01 01 01 01 .1 .1 .1 01 01 01 01 01 .9 .5 01;
        01 .3 .3 .3 .3 01 .1 .1 01 01 01 .9 .9 .9 .5;
        01 .3 .4 .6 .6 .3 .3 .1 01 01 01 .7 .9 .9 .9;
        01 .3 .3 .7 .7 .3 .3 .3 .1 01 01 01 .8 .9 .9;
        01 01 01 .6 .6 .5 01 01 .1 01 01 01 01 .5 .8;
        01 01 01 01 01 01 01 .1 .1 01 01 01 01 .5 .7;
        01 01 01 01 01 01 01 .1 01 01 01 01 01 01 01;
        .1 .1 .1 .1 .1 .1 .1 01 01 01 01 01 .6 .4 01;
        01 01 01 01 01 01 01 01 01 01 01 .3 .6 .6 01;
        01 01 01 01 01 01 01 01 01 01 01 01 01 .4 01];

costs = flipud(costs);

% road low P_tr (should avoid)
P_tr=sqrt([.99 .50 .50 .99 .99 .99 .75 .75 .99 .99 .99 .99 .99 .99 .99;
           .99 .50 .50 .99 .99 .75 .75 .75 .85 .99 .99 .99 .85 .99 .99;
           .99 .50 .50 .90 .99 .99 .75 .65 .75 .99 .99 .80 .75 .80 .95;
           .99 .99 .65 .50 .50 .99 .99 .99 .99 .99 .99 .80 .75 .75 .75;
           .99 .99 .99 .85 .50 .99 .99 .99 .75 .99 .99 .99 .90 .80 .99;
           .99 .99 .99 .75 .50 .50 .75 .50 .99 .99 .99 .99 .99 .95 .99;
           .99 .99 .99 .90 .99 .50 .50 .90 .99 .99 .99 .99 .99 .99 .99;
           .99 .99 .99 .99 .99 .75 .75 .50 .75 .99 .99 .99 .99 .99 .99;
           .99 .99 .99 .90 .99 .75 .99 .50 .50 .99 .99 .99 .99 .99 .99;
           .99 .99 .75 .75 .75 .99 .99 .99 .50 .99 .99 .99 .99 .99 .99;
           .99 .99 .99 .75 .75 .99 .99 .50 .50 .99 .99 .99 .99 .99 .99;
           .99 .99 .99 .99 .99 .99 .50 .50 .99 .99 .99 .99 .99 .80 .99;
           .50 .50 .50 .50 .50 .50 .50 .99 .99 .99 .99 .99 .75 .75 .99;
           .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .75 .75;
           .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .75 .75]);
      
P_tr = flipud(P_tr);   

%build adjacency matrix
n_rows = 15; % must be at least 2x2.
n_cols = 15;
% s = 15;
V = n_rows*n_cols; %total number of nodes
% N = V^2;
i_vals = [];
j_vals = [];
dist = [];

P_tr_thresh = .850;

tic

%create obstacles
obst = add_obstacles();

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

gplot(adj, coords, '*-') %plot graph
axis off

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
end
axis off
figure

[c, p] = plot_paths( d, best_path, cost, P_tr, coords );
figure
plot_maps(V, coords, costs, P_tr);

figure(1)

p_start = [1,1]; 
course = [0 0 16 16];
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
robot.r = 0.75;
robot.t = 0;

robot.p = p_start;               %set robot position to x_start
robot.v = [0; 0; 0];


circle(p_start(1,1),p_start(1,2),goal.r,'g');               %draw the location of x_start and x_goal

ka = 1;        %attractive gain
kr = .7;        %repulsive gain

dt = .02;        %time step size (seconds)

%%%%%%%%%%%%%%while robot position != goal:
h = draw_robot(robot);
for ii = 1:length(best_path)
    
    p_goal = coords(best_path(ii), :);
    while norm(robot.p - p_goal) > robot.r+goal.r
        %%%%%%%%%Define robot position
    %     robot.x = [robot.p; robot.v];
        map = [];
        %%%%%%%%%%%do a sensor sweep
        [ map, s ] = sensor( robot, obst, map, s, param.sensor_range, course);

        %%%%%%%%%%%Find potential function
            %attractive potential
    %         dU_a = ka*(robot.p - p_goal)*sqrt(norm(robot.p - p_goal))/norm(robot.p - p_goal);
    %         dU_a = ka*(robot.p -p_goal);
             dU_a = (robot.p - p_goal)/norm(robot.p - p_goal)*10;

    %         plot([robot.p(1), robot.p(1)-dU_a(1)], [robot.p(2), robot.p(2)-dU_a(2)], 'g' )

            %repulsive potential
            dU_r = zeros(length(map),2);
            for i=1:length(map)
                d_to_obst = norm(-map(i).p + robot.p);
                del_r = (-map(i).p + robot.p)/norm(-map(i).p + robot.p);
                dU_r(i,:) = kr*(1/param.sensor_range - 1/d_to_obst)*1/d_to_obst^2*del_r;
    %             plot([robot.p(1), robot.p(1)+dU_r(i,1)], [robot.p(2), robot.p(2)+dU_r(i,2)], 'r' )
            end

        %%%%%%%%%%%Find gradient
    %         dv = sum([-dU_a'; -dU_r],1);
    %         dtheta = atan2(dv(2), dv(1));
    %         F = [dv'; dtheta];

            F = sum([-dU_a'; -dU_r],1);

    %             F = [2, 0];

        %%%%%%%%%%%% Calculate new control input    
    %         v_d = F'/c;
    %         e_v = -robot.x(3:4) + v_d;
    %         robot.v = kv*e_v;

        %%%%%%%%%%%% Move robot for one timestep
            plot(robot.p(1), robot.p(2), 'bx')
            old_p = robot.p;

            plot([old_p(1), robot.p(1)],[old_p(2), robot.p(2)],'g', 'LineWidth', 3)

            robot = state_int(robot, F, dt);
    %         robot.p = robot.p + dt*F';

            set(h, 'Visible', 'off')
            h = draw_robot(robot);
            set(h, 'Visible', 'on')
            drawnow
    end

end







