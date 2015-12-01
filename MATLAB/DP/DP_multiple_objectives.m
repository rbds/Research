%Testing multi-objective optimization

clear
close all

P_tr_thresh = 0.75;

%build adjacency matrix
%square grid, connected right and down
s =12; % s is number of grid sections +1 (nodes will represent centers of grid sections).
V = s^2;
i_vals = [];
j_vals = [];
% k_vals = [];
for i = 1:V
   if (mod(i,s) >0) %if it isn't on the right edge of grid
    i_vals(end+1) = i; %add node to right
    j_vals(end+1) = i+1;
   end
%    if (mod(i, s) ~=1) %if node isn't on the left edge of grid
%     i_vals(end+1) = i; %add node to left
%     j_vals(end+1) = i-1;
%    end
   if (i<=(V- s))   %if vertex isn't on bottom of grid
    i_vals(end+1) = i;  %add next node down
    j_vals(end+1) = i+s;
   end
%    if (i> s) %if vertex isn't on top of grid
%     i_vals(end+1) = i;   %add next node up
%     j_vals(end+1) = i - s;
%    end
end

vals = 10*abs(randn(length(i_vals), 1)); %generate random costs
adj = sparse(i_vals, j_vals, vals, V, V);

d{V, 1} = []; %d{i}(1) is minimum cost, d{i}(2) is total P_tr, d{i}(3) is parent node
P_tr = sqrt(rand(V,1)); %Prob. of traverse associated with each node. sqrt is to bias towards higher values.

%create coordinates
coords = [1 s];
for i = 1:V-1
   coords(i+1,:) = [floor(i/s)+1, s - mod(i,s)];
end
gplot(adj, coords, '*-')

P_tr(V) = 1;
d{V}(2) = 1; %P_tr for last node is 1.
d{V}(1) = 0;
d{V}(3) = V;
for i=V-1:-1:1 %counting back from last node,
%     conns = find(adj(i,:)~=0);  %find non-zero entries in adj
     options = [];
%     %for j=1:size(conns,1)   %add all entries in d{conns} to options.
%        options(end+(1:size(d{conns},1)),:) = (d{conns} + full([adj(i, conns), 0 , 0])).*[1 P_tr(i) 1]; %total cost
%        
%        options((end - size(d{conns},1)):end, 3) = conns;
%     %end
%     
    conns = find(adj(i,:)~=0);  %find non-zero entries in adj
    for j=1:length(conns)
         l = size(d{conns(j)}(:,1),1);
         p = size(options,1);
         options((end+1):(end+l), 1) = d{conns(j)}(:,1) + adj(i, conns(j));%list all options, find cost
         if p>0
             options((p+1):end, 2) = d{conns(j)}(:,2)*P_tr(i); %calculate P_tr for options
             options((p+1):end, 3) = conns(j); %list parent node for each option.
         else
             options(:, 2) = d{conns(j)}(:,2)*P_tr(i); %calculate P_tr for options
             options(:, 3) = conns(j); %list parent node for each option.
         end
    end
%     options

    % inner front
    
    
    if isempty(d{i})
%         d{i} = options;
%         front = d{i};
        front = options;
    
        list1 = find(front(:,1) >= options(:,1));       %find list of points with greater cost
        list2 = find(front(:,2) >= options(:,2));       %find list of points with greater P_tr
        to_remove = [];     
        ind = [];

        for j=1:length(list1)
            ind = find(list1(j) == list2);      %if any entries are on both lists, remove them.
           if ~isempty(ind)
            to_remove(end+1) = ind;
           end
        end
        front(list2(to_remove),:) = [];
        l = size(options,1);
        front((end+1):(end+l), :) = options;
        
        list1 = find(front(:,1) >= options(:,1));       %find list of points with greater cost
        list2 = find(front(:,2) >= options(:,2));       %find list of points with greater P_tr
        if (length(list1) + length(list2) >0)   %check if any existing nodes are dominated
            t1 = front(:,1) <= options(:,1);
            t2 = front(:,2) <= options(:,2);
            if (isempty(find(t1==t2)))
                front(end+1,:) = options;
            end
        end
       d{i} = front;
    end %end if
    
    
    %[d(i,1), ind] = min(options(:,1));  %choose minimum option
    %par_point(i) = conns(ind);          %set parent for current node
    %d(i,2) = P_tr(par_point(i))*P_tr(i);%calculate cost for current node.
end

t = 1;
path = [1];
while (t<V-1)
    path(end+1) = d{t}(1,3);
    t = path(end);
end
path
cost = d{1}(1)
Prob_traverse = d{1}(2)
hold on
for i=1:length(path)-1
    plot(coords(path(i),1), coords(path(i),2), 'r*')
    plot([coords(path(i),1), coords(path(i+1),1)],[coords(path(i),2), coords(path(i+1),2)], 'r-')    
end
axis off
axis equal