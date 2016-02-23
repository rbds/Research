clear
close all
ha = axes('units','normalized','position',[0 0 1.1 1.1]);

uistack(ha,'bottom');

I=imread('sample_map.png');
hi = imagesc(I);
colormap gray

set(ha,'handlevisibility','off','visible','off')

axes('position',[0 0 .95 .95])
axis off
axis equal

costs = [.3 .2 .3 .5 .3 .3 .3 .3 .5 .4 .3 .4 .4 .3 .2;
        .3 .5 .5 .3 .2 .2 .2 .3 .7 .6 .3 .3 .5 01 .3;
        .2 .3 .1 .2 .4 .3 .2 .2 .7 .8 .3 .3 .4 .3 .3;
        .3 .3 .2 .1 .1 .2 .3 .2 .4 .4 .3 .6 .3 .2 .2;
        .3 .3 .5 .4 .2 .1 .1 .2 .5 .2 .8 01 .7 .6 .4;
        .3 .5 .7 .7 .3 .2 .2 .1 .1 .2 .8 01 01 01 .5;
        .3 .3 .5 .4 .3 .3 .2 .2 .1 .1 .2 .5 .8 .9 .3;
        .3 .3 .3 .3 .3 .3 .3 .2 .2 .1 .2 .2 .6 01 .4;
        .3 .3 .3 .2 .4 .3 .3 .3 .2 .1 .2 .3 .4 .6 .3;
        .3 .3 .3 .2 .5 .3 .2 .2 .1 .2 .2 .3 .3 .3 .3;
        .2 .1 .2 .2 .2 .1 .1 .1 .5 .5 .3 .3 .3 .3 .3;
        .2 .2 .1 .1 .1 .2 .2 .2 .5 .3 .3 .3 .3 .4 .3;
        .1 .1 .1 .1 .1 .2 .2 .5 .5 .3 .3 .6 .6 .4 .3;
        .1 1 .1 .1 .1 .2 .2 .2 .3 .3 .3 .6 .6 .4 .3;
        .2 .3 .3 .3 .3 .3 .3 .3 .3 .3 .3 .6 .6 .4 .3];
    
% costs = flipud(costs);
costs = costs.^2;
    
P_tr =sqrt([.90 .50 .50 .75 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99;
       .90 .50 .50 .99 .99 .99 .99 .90 .75 .50 .60 .10 .10 .10 .90;
       .90 .80 .45 .75 .99 .99 .99 .90 .99 .50 .10 .10 .10 .70 .90;
       .99 .80 .90 .50 .50 .75 .99 .99 .05 .05 .10 .50 .50 .50 .99;
       .99 .95 .10 .85 .75 .50 .60 .80 .75 .99 .99 .99 .50 .05 .99;
       .99 .90 .10 .25 .99 .95 .75 .50 .60 .99 .99 .99 .99 .99 .99;
       .99 .99 .10 .90 .99 .99 .99 .90 .50 .65 .99 .75 .99 .99 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .90 .50 .60 .99 .75 .99 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .90 .50 .90 .99 .99 .99 .99;
       .99 .99 .99 .99 .75 .99 .99 .99 .60 .60 .90 .99 .99 .99 .99;
       .50 .50 .60 .60 .60 .60 .50 .50 .65 .75 .99 .99 .99 .99 .99;
       .60 .60 .40 .50 .50 .50 .60 .50 .99 .99 .99 .99 .99 .80 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .85 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .85 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .85 .99]);
P_tr = flipud(P_tr);   

V = 15*15;
s = 15;
N = V^2;
i_vals = [];
j_vals = [];

P_tr = repmat(P_tr, V, 1);

P_tr_thresh = 0.3; 



tic

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
axis off

[adj_i, adj_j, adj_v] = find(adj); %access rows and columns of adjacency matrix.

P_tr(N-V:N) = 1;
d{N,1} = []; % array to list parent paths

for i=(N-V+1):(N) %create d vector to store cost, P_tr, parents for each entry in adj.
   d{i,1} = [0 1 i];
%     d{i,1} = [0 1];
%     d{i,2}(1) = i;
end

for i=1:V
   d{i*V} = [0 1 i*V]; 
end

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
        
        for k=1:size(new_options,1) %if option violates P_tr, set penalty cost
           if new_options(k,2) < P_tr_thresh
            new_options(k,1) = new_options(k,1) + 1000;
           end
        end
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
% Ptr = prod(P_tr(best_path));
Ptr = d{1}(1,2)
time

hold on
for i=1:length(best_path)-1  %plot path
    plot(coords(best_path(i),1), coords(best_path(i),2), 'r*')
    plot([coords(best_path(i),1), coords(best_path(i+1),1)],[coords(best_path(i),2), coords(best_path(i+1),2)], 'r-', 'LineWidth', 4)    
end
axis off
% axis equal