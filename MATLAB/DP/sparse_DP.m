%Testing multi-objective optimization

clear
close all

tic

P_tr_thresh = 0.2;
% times = zeros(1,10);

% for z = 2:10
%build adjacency matrix
s =10;
V = s^2;
N = V^2;
i_vals = [];
j_vals = [];

minimum = 0.9;
maximum = 1.0;
P_tr = (maximum - minimum)*rand(V, 1) + minimum; %Prob. of traverse associated with each node. sqrt is to bias towards higher values.
P_tr = repmat(P_tr, V, 1);
% costs = 10*abs(randn(V,1));
costs = [.1 .7 .8 .7 .6 .2 .2 .3 .2 .1
       .5 .9 .7 .7 .3 .4 .5 .3 .1 .1
       .7 .6 .3 .2 .7 .5 .3 .1 .1 .1
       .2 .4 .9 .8 .5 .2 .2 .2 .1 .1
       .8 .9 .7 .4 .2 .1 .2 .3 .1 .8
       .2 .1 .1 .2 .8 .5 .4 .4 .1 .8
       .1 .6 .5 .6 .5 .4 .4 .2 .1 .1
       .3 .2 .1 .1 .1 .1 .2 .3 .1 .1
       .2 .2 .1 .1 .1 .2 .1 .1 .1 .1
       .1 .2 .2 .2 .4 .2 .1 .1 .1 .1];
costs = reshape(costs, 1, 100);

for i = 1:V    %for each row in adjacency matrix
   if (mod(i,s) >0) %if it isn't on the right edge of grid 
    j_vals(end+1) = i; %add node to right
    i_vals(end+1) = i+1;
   end

   if (mod(i, s) ~=1) %if node isn't on the left edge of grid
    j_vals(end+1) = i; %add node to left
    i_vals(end+1) = i-1;
   end
   
   if (i> s) %if vertex isn't on top of grid
    j_vals(end+1) = i ;   %add next node up
    i_vals(end+1) = i - s;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1 - s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1 -s;
       end
   end
   
   if (i<=(V- s))   %if vertex isn't on bottom of grid
    j_vals(end+1) = i ;  %add next node down
    i_vals(end+1) = i+s ;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1+ s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1+s;
       end
   end   
end



% %create coordinates
coords = [];
for i = 1:N
   block_no = ceil(i/V);
   V_no = i - (block_no-1)*V;

   row = mod(V_no,s);
   col = ceil(V_no/s);
      if (row==0) row = s; end 
   coords(end+1,:) = [row, col]; 
end

%create adjacency matrix
% vals = 10*abs(randn(length(i_vals), 1));  %generate random costs
vals = zeros(size(i_vals));
for i=1:length(i_vals)
    vals(i) = mean([costs(i_vals(i)), costs(j_vals(i))]);
end
a= sparse(i_vals, j_vals, vals); %one section of the adjacency matrix
adj = sparse(V^2, V^2);
for i=1:V-1 %assemble adjacency matrix from sections
   adj((i-1)*V+1:(i-1)*V+V, i*V+1:i*V+V) = a;
end
%  spy(adj)
gplot(adj, coords, '*-') %plot graph

[adj_i, adj_j, adj_v] = find(adj); %access rows and columns of adjacency matrix.

P_tr(V^2-V:V^2) = 1;
%d{i,1}(:,1) is minimum cost, d{i,1}(:,2) is total P_tr, d{i,1}(:,3) is path
%represented by that node.
d{N,2} = []; % array to list parent paths
% for i=1:N
%    d{i,2} = zeros(V);
% end
for i=(N-V+1):(N) %create d vector to store cost, P_tr, parents for each entry in adj.
   d{i,1} = [0 1 i];
%     d{i,1} = [0 1];
%     d{i,2}(1) = i;
end

for i=1:V
   d{i*V} = [0 1 i*V]; 
end


% d{N} = [0 1 N];
% d = d'; %make column vector

%For each column in adjacency matrix, find all connections.
%for each connection:
    %populate P_tr and cost
    %add all options to list in connected node
    %run pareto front for connected code

t = zeros(V, 2*V);
for i=1:V
    r = (1+(i-1)*V):i*V;
    t(i,i:(i+V-1)) = r; 
end

rows_to_do = [];
for i=1:2*V
   l = t(find(t(:,i)),i);
   rows_to_do(end+1:end+(length(l))) = l;
end
rows_to_do = fliplr(rows_to_do);

    
% for i=N:-1:1 %counting back from last populated column in adjacency matrix,
for i=rows_to_do
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
%           new_options = [d{i,1}(:,1) + adj(row,col), d{i,1}(:,2)*P_tr(V_row)];      
                %these possibilities come from a path through the current
                %node, i.
        
        for k=1:size(new_options,1) %if option violates P_tr, set penalty cost
           if new_options(k,2) < P_tr_thresh
            new_options(k,1) = new_options(k,1) + 1000;
           end
        end
    options = [options; new_options];

% build front 
%     inds = [];
    [front, inds] = prto(options);
%     front = prto(options);
   
    %put pareto front into list for connected node
    if isempty(front) 
        d{conns(j),1} = options;
%         d{conns(j),2} = repmat(i, size(options,1),1);
    else
        d{conns(j),1} = front;
%         d{conns(j),2} = d{conns(j),2}(inds);
    end
  
    end
end


time = toc;
% times(z) = time;

% end
% d{1}

% %extract best path
best_path = 1;
t = 1;
while (mod(t,V)>0) %while on a node that isn't the last node
    t = d{t}(1,3);    %find minimum cost path at node t
    best_path(end+1) = t;
    
%     t = path(end);
end
best_path
cost = d{1}(1)
Ptr = prod(P_tr(best_path))
time

% hold on
% for i=1:size((best_path),1)-1  %plot path 
%     plot(coordsbest_pathbp(i),1, coords(best_path(i),2), 'r*')
%     plot([coords(best_path(i),1), coords(best_path(i+1),1)],[coords(best_path(i),2), coords(best_path(i+1),2)], 'r-')    
% end
% axis off
% axis equal

hold on
for i=1:length(best_path)-1  %plot path
    plot(coords(best_path(i),1), coords(best_path(i),2), 'r*')
    plot([coords(best_path(i),1), coords(best_path(i+1),1)],[coords(best_path(i),2), coords(best_path(i+1),2)], 'r-', 'LineWidth', 4)    
end
axis off
axis equal