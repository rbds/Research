function [ best_path ] = extract_best_path(d, start_node, end_node, eps_tr )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% %extract best path
best_path = start_node;
t = start_node;
while (mod(t,end_node)>0) %while on a node that isn't the last node
    [y, best] = min(d{t}(:,1));
    
    if d{t}(best,2) > eps_tr
        t = d{t}(best,3);    %find minimum cost path at node t
    else    
        [y,i] = find(d{t}(:,2)>.5);
        if isempty(y)
           t = d{t}(end,3);
        else
           t = d{t}(y(1),3);  
        end
    end
    
    
    
    best_path(end+1) = t;
    
%     t = path(end);
end
best_path;

end

