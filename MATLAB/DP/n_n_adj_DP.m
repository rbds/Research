clear
clf
ha = axes('units','normalized','position',[0 0 1.1 1.1]);

uistack(ha,'bottom');

I=imread('sample_map.png');
hi = imagesc(I);
colormap gray

set(ha,'handlevisibility','off','visible','off')

axes('position',[0 0 .95 .95])
axis off
axis equal

costs = [.3 .2 .2 .5 .3 .3 .3 .3 .5 .4 .3 .4 .4 .3 .2;
        .3 .5 .1 .3 .2 .2 .2 .3 .7 .6 .3 .3 .5 .4 .3;
        .2 .3 .1 .2 .4 .3 .2 .2 .7 .8 01 01 01 .3 .3;
        .3 .3 .2 .1 .1 .2 .3 .2 .4 .4 01 01 .4 .2 .2;
        .3 .3 .5 .4 .2 .1 .1 .2 .5 .2 .8 01 .7 .3 .2;
        .3 .5 .7 .7 .3 .2 .2 .1 .1 .2 .8 .2 .3 .2 .2;
        .3 .3 .5 .4 .3 .3 .2 .2 .1 .1 .2 .2 .3 .7 .3;
        .3 .3 .3 .3 .7 .7 .7 .2 .2 .1 .2 .2 .6 .2 .4;
        .3 .3 .3 .2 .7 .7 .7 .3 .2 .1 .2 .2 .4 .6 .3;
        .3 .3 .3 .2 .7 .7 .2 .2 .1 .2 .2 .2 .3 .3 .3;
        .2 .3 .2 .2 .2 .1 .1 .5 .5 .2 .3 2 .3 .3 .3;
        .2 .2 .1 .1 .1 .2 .2 .2 .5 .3 .3 .2 .3 .4 .3;
        .1 .1 .1 .1 .1 .2 .2 .5 .5 .3 .3 .6 .6 .4 .3;
        .1 .1 .1 .1 .1 .2 .2 .2 .3 .3 .3 .6 .6 .4 .3;
        .2 .3 .3 .3 .3 .3 .3 .3 .3 .3 .3 .6 .6 .4 .3];
    
costs = flipud(costs);
% costs = costs.^2;
    
P_tr =([.90 .50 .50 .75 .99 .99 .99 .99 .99 .99 .99 .99 .99 .80 .99;
       .90 .50 .50 .99 .99 .99 .99 .99 .99 .80 .60 .75 .95 .90 .99;
       .90 .80 .45 .75 .99 .99 .99 .90 .99 .90 .50 .50 .95 .99 .99;
       .99 .99 .90 .50 .40 .40 .99 .99 .99 .99 .70 .60 .90 .99 .99;
       .99 .95 .90 .85 .45 .40 .60 .80 .75 .99 .99 .99 .95 .95 .99;
       .99 .90 .90 .25 .99 .95 .75 .50 .60 .99 .99 .99 .99 .99 .99;
       .99 .99 .90 .90 .99 .99 .99 .90 .50 .65 .99 .99 .99 .99 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .90 .50 .60 .99 .99 .99 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .90 .50 .90 .99 .99 .99 .99;
       .99 .99 .99 .99 .75 .99 .99 .99 .60 .60 .90 .99 .99 .99 .99;
       .50 .50 .60 .60 .60 .60 .50 .50 .65 .75 .99 .99 .99 .99 .99;
       .60 .60 .40 .50 .40 .50 .60 .50 .99 .99 .99 .99 .99 .80 .99;
       .60 .60 .40 .50 .40 .50 .60 .50 .99 .99 .99 .99 .99 .80 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .85 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .85 .99]);
P_tr = flipud(P_tr);   

V = 15*15;
s = 15;
N = V^2;
i_vals = [];
j_vals = [];

P_tr = repmat(P_tr, V, 1);

P_tr_thresh = 0.25; 



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
%        if (mod(i,s) >0) %if it isn't on the right edge of grid 
%         j_vals(end+1) = i; %add node to diagonal right
%         i_vals(end+1) = i+1 - s;
%        end
% 
%        if (mod(i, s) ~=1) %if node isn't on the left edge of grid
%         j_vals(end+1) = i; %add node to diagonal left
%         i_vals(end+1) = i-1 -s;
%        end
   end
   
   if (i<=(V- s))   %if vertex isn't on bottom of grid
    j_vals(end+1) = i ;  %add next node down
    i_vals(end+1) = i+s ;
%        if (mod(i,s) >0) %if it isn't on the right edge of grid 
%         j_vals(end+1) = i; %add node to diagonal right
%         i_vals(end+1) = i+1+ s;
%        end
% 
%        if (mod(i, s) ~=1) %if node isn't on the left edge of grid
%         j_vals(end+1) = i; %add node to diagonal left
%         i_vals(end+1) = i-1+s;
%        end
   end   
end

% %create coordinates
coords = [];
for i = 1:V
    row = mod(i,s);
   col = ceil(i/s);
      if (row==0) row = s; end 
   coords(end+1,:) = [row, col]; 
end

%create adjacency matrix
vals = zeros(size(i_vals));
for i=1:length(i_vals)
    vals(i) = mean([costs(i_vals(i)), costs(j_vals(i))]);
end
adj= sparse(i_vals, j_vals, vals); %one section of the adjacency matrix

gplot(adj, coords, '*-') %plot graph
axis off

[adj_i, adj_j, adj_v] = find(adj); %access rows and columns of adjacency matrix.

P_tr(V) = 1;
d{V,1} = []; %create d vector to store cost, P_tr, parents for each entry in adj.

d{V} = [0 1 i*V]; 

%order of rows to do.
t = zeros(s, 2*s);
for i=1:s
    r = ((i-1)*s + 1):i*s;  % list indices of row
    t(i,i:(i+s-1)) = r;     %shift indices over
end
rows_to_do = [];
for i=1:2*s
   l = t(find(t(:,i)),i);  %pull columns of shifted matrix
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