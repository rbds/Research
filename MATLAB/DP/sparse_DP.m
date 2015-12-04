%Testing multi-objective optimization

clear
close all

P_tr_thresh = 0.4;

%build adjacency matrix
%square grid, connected right and down
s =3;
V = s^2;
N = V^2;
i_vals = [];
j_vals = [];
vals = [];
% k_vals = [];

minimum = 0.9;
maximum = 1.0;
P_tr = (maximum - minimum)*rand(V, 1) + minimum;%Prob. of traverse associated with each node. sqrt is to bias towards higher values.
costs = 10*abs(randn(V,1));

for i = 1:V    %letter
   if (mod(i,s) >0) %if it isn't on bottom edge of grid 
    i_vals(end+1) = i; %add node to bottom
    j_vals(end+1) = i+1+V;
    vals(end+1) = (costs(i) + costs(i+1));
   end

   if (mod(i, s) ~=1) %if node isn't on top edge of grid
    i_vals(end+1) = i; %add node to top
    j_vals(end+1) = i-1 +V;
    vals(end+1) = (costs(i) + costs(i-1));
   end
   
   if (i> s) %if vertex isn't on left of grid
    i_vals(end+1) = i ;   %add next node left
    j_vals(end+1) = i - s + V;
    vals(end+1) = (costs(i) + costs(i-s));
       if (mod(i,s) >0) %if it isn't on the bottom edge of grid 
        i_vals(end+1) = i; %add node to diagonal right
        j_vals(end+1) = i+1+V - s;
        vals(end+1) = (costs(i) + costs(i+1-s));
       end
       
       if (mod(i, s) ~=1) %if node isn't on the top edge of grid
        i_vals(end+1) = i; %add node to diagonal left
        j_vals(end+1) = i-1 +V -s;
        vals(end+1) = (costs(i) + costs(i-1-s));
       end
   end
   
   if (i<=(V- s))   %if vertex isn't on right of grid
    i_vals(end+1) = i ;  %add next node right
    j_vals(end+1) = i+s + V;
    vals(end+1) = (costs(i) + costs(i+s));
       if (mod(i,s) >0) %if it isn't on the top edge of grid 
        i_vals(end+1) = i; %add node to diagonal right
        j_vals(end+1) = i+1+V + s;
        vals(end+1) = (costs(i) + costs(i+1+s));
       end

       if (mod(i, s) ~=1) %if node isn't on the bottom edge of grid
        i_vals(end+1) = i; %add node to diagonal left
        j_vals(end+1) = i-1 +V +s;
        vals(end+1) = (costs(i) + costs(i-1+s));
       end
   end
  
end


% vals = ones(length(i_vals),1);
% vals = 10*abs(randn(length(i_vals), 1));  %generate random costs
a= sparse(i_vals, j_vals, vals); %create small sparse matrix
z= sparse(V,V); %blank sparse matrix

adj = [a z z; z a z; z z a; z z z z]; %full adjacency matrix
% spy(adj)




% Coordinates aren't created now because size(adj) is V^2 x V^2, but plot
% should only be sxs.

% %create coordinates
coords = [];


% coords = [1,V];
for i = 1:N
   block_no = ceil(i/V);
   V_no = i - (block_no-1)*V;

   row = mod(V_no,s);
   col = ceil(V_no/s);
      if (row==0) row = s; end
   coords(end+1,:) = [row, col];
end
gplot(adj, coords, '*-')


ind_last = V^4 - V - 1; %index of the end node
col_last = N;   %column of last node
row_last = N-V-1;   %row of last node
V_last = [row_last, col_last ];
num = nnz(adj); %total number of non-zero values in adjacency matrix
[adj_i, adj_j, adj_v] = find(adj); %access rows and columns of adjacency matrix.

P_tr(end) = 1;
%d{i}(1) is minimum cost, d{i}(2) is total P_tr, d{i}(3) is parent node
d{N,1} = [];
for i=(N-V+1):(N) %create d vector to store cost, P_tr, parents for each entry in adj.
   d{i,1} = [0 1 N];
end

%calculate costs, P_tr for current vertex
for i=N:-1:1 %counting back from last populated column in adjacency matrix,
% list_of_nodes_to_visit = V;
% while (~isempty(list_of_nodes_to_visit)  %While the list of nodes to try is not empty
%     [col, list_of_nodes_to_visit] = pop(list_of_nodes_to_visit);         %column of current vertex
    col = i;  
        V_col = mod(col, V);
        if (V_col==0) V_col = V; end
    conns = find(adj(:,col)~=0);    %find entries in column col
    options = [];
    for j=1:length(conns)   %for each connection,
        row = conns(j);             %row of current connection
        V_row = mod(row, V);
        if (V_row==0) V_row = V; end 
        l = size(d{i}(:,1),1);
        p = size(options, 1);
        options(end+1:end+l,1) = d{i}(:,1) + adj(row,col); %cost of each one of d{i} = adj(row,col)
               
        if p>0
           options((p+1):end, 2) = d{i}(:,2)*P_tr(V_row); %calculate P_tr for options
           options((p+1):end, 3) = i; %list parent node for each option.
        else
           options(:, 2) = d{i}(:,2)*P_tr(V_row); %calculate P_tr for options
           options(:, 3) = i; %list parent node for each option.
        end 
        
        for k=1:size(options,1) %if option violates P_tr, set penalty cost
           if options(k,2) < P_tr_thresh
            options(k,1) = options(k,1) + 1000;         
           end
        end
    front = options(1,:);
%     d{conns(j)}(end+1,:) = front;  %populate cost, P_tr, parent.
   
% inner front      
    for k = 2:size(options,1)
       list1 = find(front(:,1) > options(k,1));       %find list of points with greater cost
       list2 = find(front(:,2) > options(k,2));       %find list of points with greater P_tr
    end
    for k=2:size(options,1)
        list1 = find(front(:,1) > options(k,1));       %find list of points with greater cost
        list2 = find(front(:,2) > options(k,2));       %find list of points with greater P_tr
        to_remove = [];
        ind = [];

        for l=1:length(list1)
            ind = find(list1(l) == list2);
           if ~isempty(ind)
            to_remove(end+1) = ind;
           end
        end
        front(list2(to_remove),:) = [];

        if (length(list1) + length(list2) >0)
            t1 = front(:,1) <= options(k,1);
            t2 = front(:,2) < options(k,2);
            if (isempty(find(t1==t2)))
                front(end+1,:) = options(k,:);
            end
        end
    end
    len = size(front, 1);
    d{conns(j)}(end+1:end+len,:) = front;
    end
end

t = 1;
% [val, ind] = min(d{1}(:,1)); %find minimum cost parent at node 1.
best_path = [1];
while (t<N)
    [~, ind] = min(d{t}(:,1));    %find minimum cost path at node t
    best_path(end+1) = d{t}(ind,3);
    t = best_path(end);
end
% path(end+1) = V
[~, ind] = min(d{1}(:,1));
cost = d{1}(ind,1)
Prob_traverse = d{1}(ind,2)
hold on
for i=1:length(best_path)-1  %plot path - STILL NEED TO ADD COORDS
    plot(coords(best_path(i),1), coords(best_path(i),2), 'r*')
    plot([coords(best_path(i),1), coords(best_path(i+1),1)],[coords(best_path(i),2), coords(best_path(i+1),2)], 'r-')    
end
axis off
axis equal