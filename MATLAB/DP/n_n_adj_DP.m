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

costs =[.2 .1 .1 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2;
        .2 .1 .1 .2 .2 .3 .7 .6 .2 .2 .2 .2 .3 .2 .2;
        .2 .1 .1 .2 .2 .2 .2 .8 .7 .6 .2 .3 .6 .5 .4;
        .2 .2 .1 .1 .2 .2 .3 .3 .7 .7 .2 .3 .5 .5 .5;
        .2 .2 .2 .1 .1 .2 .2 .2 .5 .6 .2 .2 .2 .3 .2;
        .2 .5 .7 .7 .1 .1 .1 .2 .2 .2 .2 .2 .9 .5 .2;
        .2 .3 .5 .7 .6 .2 .1 .1 .2 .2 .2 .9 .9 .9 .5;
        .2 .3 .4 .6 .6 .3 .3 .1 .2 .2 .2 .7 .9 .9 .9;
        .2 .3 .3 .2 .4 .3 .3 .3 .1 .2 .2 .2 .8 .9 .9;
        .2 .3 .3 .2 .5 .6 .2 .2 .1 .2 .2 .2 .2 .5 .8;
        .2 .2 .2 .2 .2 .2 .2 .1 .1 .2 .2 .2 .2 .5 .7;
        .2 .2 .2 .2 .2 .2 .2 .1 .2 .2 .2 .2 .2 .2 .2;
        .1 .1 .1 .1 .1 .1 .1 .2 .2 .2 .2 .2 .6 .4 .2;
        .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .3 .6 .6 .2;
        .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .4 .2];
    
% costs =[.2 .1 .1 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2;
%         .2 .1 .1 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2;
%         .2 .1 .1 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2;
%         .2 .2 .1 .1 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2;
%         .2 .2 .2 .1 .1 .2 .2 .2 .2 .2 .2 .2 .2 .3 .2;
%         .2 .2 .2 .2 .1 .1 .1 .2 .2 .2 .2 .2 .9 .5 .2;
%         .2 .2 .2 .2 .2 .2 .1 .1 .2 .2 .2 .9 .9 .9 .5;
%         .2 .2 .2 .2 .2 .2 .2 .1 .2 .2 .2 .7 .9 .9 .9;
%         .2 .3 .3 .2 .4 .3 .3 .3 .1 .2 .2 .2 .8 .9 .9;
%         .2 .3 .3 .2 .5 .6 .2 .2 .1 .2 .2 .2 .2 .5 .8;
%         .2 .2 .2 .2 .2 .2 .2 .1 .1 .2 .2 .2 .2 .5 .7;
%         .2 .2 .2 .2 .2 .2 .2 .1 .2 .2 .2 .2 .2 .2 .2;
%         .1 .1 .1 .1 .1 .1 .1 .2 .2 .2 .2 .2 .2 .2 .2;
%         .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2;
%         .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .2 .4 .2];
    
    
costs = flipud(costs);
% costs = costs.^4;

% % road low P_tr (should avoid)
P_tr=sqrt([.99 .50 .50 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99;
           .99 .50 .50 .99 .99 .99 .99 .99 .85 .99 .99 .99 .85 .99 .99;
           .99 .50 .50 .90 .99 .99 .90 .65 .75 .99 .99 .80 .75 .80 .95;
           .99 .99 .65 .50 .50 .75 .99 .75 .75 .99 .99 .80 .75 .75 .75;
           .99 .99 .99 .85 .50 .90 .60 .80 .75 .99 .99 .99 .90 .80 .99;
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


% % road high P_tr (should follow)
% P_tr=sqrt([.99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99;
%            .99 .99 .99 .99 .99 .99 .99 .99 .85 .99 .99 .99 .99 .99 .99;
%            .99 .99 .99 .99 .99 .99 .90 .65 .75 .99 .99 .80 .75 .80 .95;
%            .99 .99 .99 .99 .99 .75 .99 .75 .75 .99 .99 .80 .75 .75 .75;
%            .99 .99 .99 .85 .99 .90 .60 .80 .75 .99 .99 .99 .90 .80 .99;
%            .99 .99 .99 .75 .99 .99 .75 .50 .99 .99 .99 .99 .99 .99 .99;
%            .99 .99 .99 .90 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99;
%            .99 .99 .99 .99 .99 .75 .99 .99 .75 .99 .99 .99 .99 .99 .99;
%            .99 .99 .99 .90 .99 .75 .99 .99 .99 .99 .99 .99 .99 .99 .99;
%            .99 .99 .75 .75 .75 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99;
%            .99 .99 .99 .75 .75 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99;
%            .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .80 .99;
%            .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .75 .75 .99;
%            .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .75 .75;
%            .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .75 .75]);

       
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
P_tr_thresh = 0.0;


% minimum = 0.9;
% maximum = 1.0;
% P_tr = (maximum - minimum)*rand(V, 1) + minimum; %Prob. of traverse associated with each node. sqrt is to bias towards higher values.
% P_tr = repmat(P_tr, V, 1);
% costs = 10*abs(randn(V,1));
% 
% P_tr = repmat(P_tr, V, 1);


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
        
%         for k=1:size(new_options,1) %if option violates P_tr, set penalty cost
%            if new_options(k,2) < P_tr_thresh %&& new_options(k,1) < 1000
%             new_options(k,1) = new_options(k,1) + 1e6*(P_tr_thresh -new_options(k,2));
%            end
%         end
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
