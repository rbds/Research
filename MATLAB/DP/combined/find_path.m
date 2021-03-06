function [d, best_path] =  find_path(start_node, target_node, env, V, d, adj, coords, P_tr, P_tr_thresh)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%put nodes in topological order.
if strcmp(env, 'pipeline')
%     start_node = 11;
%     target_node = 988;
    dists = zeros(V,1);
    for jj = 1:V
        dists(jj) = norm([coords(jj,1) coords(jj,2)] - [coords(target_node,1) coords(target_node,2)]);
    end
    [y, rows_to_do] = sort(dists, 'ascend');
elseif strcmp(env, 'field')
    dists = zeros(V,1);
    for jj = 1:V
        dists(jj) = norm([coords(jj,1) coords(jj,2)] - [coords(target_node,1) coords(target_node,2)]);
    end
    [y, rows_to_do] = sort(dists, 'ascend');

else
%     start_node = 1;
%     target_node = V;
    rows_to_do = V:-1:1;
    rows_to_do  = rows_to_do';
end
d{target_node} = [0 1 target_node];

% for i=V:-1:1 %counting back from last populated column in adjacency matrix,
for ii=rows_to_do'
    col = ii;
        V_col = mod(col, V);
        if (V_col==0) V_col = V; end 
    conns = find(adj(:,col)~=0);    %find entries in column col
    
    if mod(ii,100)==0
       disp('on column: ')
       disp(ii)
    end
    
    for jj=1:length(conns)   %for each connection,
        if isempty(d{conns(jj)})
            options = [];
        else
            options = d{conns(jj),1}; %pull current pareto front
        end
        row = conns(jj);             %row of current connection
        V_row = mod(row, V);
        if (V_row==0) V_row = V; end 

        new_options = [d{ii,1}(:,1) + adj(row,col), d{ii,1}(:,2)*P_tr(V_row), repmat(ii, size(d{ii,1}(:,1)))];   %list possible new paths
        
    options = [options; new_options];
    [front, inds] = prto(options);
       
    if isempty(front) 
        d{conns(jj),1} = options;

    else
        d{conns(jj),1} = front;
    end
  
    end
end

time = toc;
hold on
% bad_paths = 0;
for i=1:length(d)
    bad_paths = find(d{i}(:,2)<P_tr_thresh);
    d{i}(bad_paths, 1) = d{i}(bad_paths,1) + 1e6;
end
best_path = extract_best_path(d, start_node, target_node, P_tr_thresh);
cost = min(d{1}(:,1))
Ptr = prod(P_tr(best_path))
% Ptr = d{1}(1,2)
time

end

