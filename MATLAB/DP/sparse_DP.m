%Testing multi-objective optimization

clear
close all

P_tr_thresh = 0.4;

%build adjacency matrix
%square grid, connected right and down
s =2;
V = s^2;
N = V^2;
i_vals = [];
j_vals = [];
% k_vals = [];

for i = 1:V    %letter
   if (mod(i,s) >0) %if it isn't on the right edge of grid 
    i_vals(end+1) = i; %add node to right
    j_vals(end+1) = i+1+V;
   end

   if (mod(i, s) ~=1) %if node isn't on the left edge of grid
    i_vals(end+1) = i; %add node to left
    j_vals(end+1) = i-1 +V;
   end
   
   if (i> s) %if vertex isn't on top of grid
    i_vals(end+1) = i ;   %add next node up
    j_vals(end+1) = i - s + V;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        i_vals(end+1) = i; %add node to diagonal right
        j_vals(end+1) = i+1+V - s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        i_vals(end+1) = i; %add node to diagonal left
        j_vals(end+1) = i-1 +V -s;
       end
   end
   
   if (i<=(V- s))   %if vertex isn't on bottom of grid
    i_vals(end+1) = i ;  %add next node down
    j_vals(end+1) = i+s + V;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        i_vals(end+1) = i; %add node to diagonal right
        j_vals(end+1) = i+1+V + s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        i_vals(end+1) = i; %add node to diagonal left
        j_vals(end+1) = i-1 +V +s;
       end
    
   end

      
end

minimum = 0.9;
maximum = 1.0;
vals = ones(length(i_vals),1);
% vals = 10*abs(randn(length(i_vals), 1));  %generate random costs
% adj = sparse(i_vals, j_vals, vals, V^2, V^2);
a= sparse(i_vals, j_vals, vals);
z= sparse(V,V);

adj = [a z z; z a z; z z a; z z z z]; %
spy(adj)

% d{V*V, 1} = []; %d{i}(1) is minimum cost, d{i}(2) is total P_tr, d{i}(3) is parent node
P_tr = (maximum - minimum)*rand(V, 1) + minimum;%Prob. of traverse associated with each node. sqrt is to bias towards higher values.
costs = 10*abs(randn(V,1));
% Coordinates aren't created now because size(adj) is V^2 x V^2, but plot
% should only be sxs.
% %create coordinates
% % coords = [1 V];
% % for i = 1:V-1
% %    coords(i+1,:) = [floor(i/s)+1, s - mod(i,s)];
% % end
% % % gplot(adj, coords, '*-')


ind_last = V^4 - V - 1;
col_last = N;
row_last = N-V-1;
V_last = [row_last, col_last ];
num = nnz(adj);
[adj_i, adj_j, adj_v] = find(adj); %access rows and columns of adjacency matrix.
% adj_i = flipud(adj_i);
% adj_j = flipud(adj_j);
% adj_v = flipud(adj_v);

P_tr(end) = 1;
% d{ind_last}(2) = 1; %P_tr for last node is 1.
% d{ind_last}(1) = 0; %cost for last node is 0.
% d{ind_last}(3) = ind_last;
% d((num-(V-1)):num,:) = repmat([0 1 num], V, 1); %set cost, P_tr, parent for end node.
for i=(N-V+1):(N) %create d vector to store cost, P_tr, parents for each entry in adj.
   d{i,1} = [0 1 num];
end

for i=N:-1:1 %counting back from last populated row in adjacency matrix,
    col = adj_j(i);
        V_col = mod(col, V);
        if (V_col==0) V_col = V; end
    conns = find(adj(:,col)~=0);    %find entries in column col
    options = [];
    for j=1:length(conns)   %for each connection,
        row = conns(j);
        V_row = mod(row, V);
        if (V_row==0) V_row = V; end 
        
        l = size(d{i}(:,1),1);
        p = size(options, 1);
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
    d{conns(j)}(end+1,:) = front;  %populate cost, P_tr, parent.
        
    end
    
   % inner front     
    
    
%     for j = 2:size(options,1)
%        list1 = find(front(:,1) > options(j,1));       %find list of points with greater cost
%        list2 = find(front(:,2) > options(j,2));       %find list of points with greater P_tr
%     end
%         for j=2:size(options,1)
%             list1 = find(front(:,1) > options(j,1));       %find list of points with greater cost
%             list2 = find(front(:,2) > options(j,2));       %find list of points with greater P_tr
%             to_remove = [];
%             ind = [];
% 
%             for k=1:length(list1)
%                 ind = find(list1(k) == list2);
%                if ~isempty(ind)
%                 to_remove(end+1) = ind;
%                end
%             end
%             front(list2(to_remove),:) = [];
% 
%             if (length(list1) + length(list2) >0)
%                 t1 = front(:,1) <= options(j,1);
%                 t2 = front(:,2) <= options(j,2);
%                 if (isempty(find(t1==t2)))
%                     front(end+1,:) = options(j,:);
%                 end
%             end
%         end
%     d{i} = front;
        
%         d{conns(j)}(end+1,:) = options;
%         d(conns(j), :) = [d(i,1) + costs(row), d(i,2)*P_tr(row), i];
    
    
%     conns = find(adj(i,:)~=0);  %find non-zero entries in adj
%     options = [];
%     for j=1:length(conns)
%      l = size(d{conns(j)}(:,1),1);
%      p = size(options,1);
%      options((end+1):(end+l), 1) = d{conns(j)}(:,1) + adj(i, conns(j));%list all options, find cost
%      if p>0
%          options((p+1):end, 2) = d{conns(j)}(:,2)*P_tr(i); %calculate P_tr for options
%          options((p+1):end, 3) = conns(j); %list parent node for each option.
%      else
%          options(:, 2) = d{conns(j)}(:,2)*P_tr(i); %calculate P_tr for options
%          options(:, 3) = conns(j); %list parent node for each option.
%      end
%     end
%  
%     for k=1:size(options,1) %if option violates P_tr, set penalty cost
%         if options(k,2) < P_tr_thresh
%             options(k,1) = 1000;         
%         end
%     end
% 
%             front = options(1,:);

end

t = 1;
% [val, ind] = min(d(t,1)); %find minimum cost path at node 1.
path = [1];
while (t<V-1)
    [~, ind] = min(d(t,1));    %find minimum cost path at node t
    path(end+1) = d(t,3);
    t = path(end);
end
path(end+1) = V
[~, ind] = min(d{1}(:,1));
cost = d{1}(ind,1)
Prob_traverse = d{1}(ind,2)
hold on
for i=1:length(path)-1  %plot path
    plot(coords(path(i),1), coords(path(i),2), 'r*')
    plot([coords(path(i),1), coords(path(i+1),1)],[coords(path(i),2), coords(path(i+1),2)], 'r-')    
end
axis off
axis equal