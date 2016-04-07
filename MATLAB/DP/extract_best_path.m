function [ best_path ] = extract_best_path(d, start_node, end_node )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% %extract best path
best_path = start_node;
t = start_node;
while (mod(t,end_node)>0) %while on a node that isn't the last node
    [bv, best] = min(d{t}(:,1));
    t = d{t}(best,3);    %find minimum cost path at node t
    best_path(end+1) = t;
    
%     t = path(end);
end
best_path;

end

