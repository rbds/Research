% %extract best path
% best_path = 1;
% t = 1;

step_no = 1;
paths = 1;

best_path = 1+V;
t = best_path;
while (mod(t,V)>0) %while on a node that isn't the last node
    t = d{t}(1,3);    %find minimum cost path at node t
    best_path(end+1) = t;
    
%     t = path(end);
end
best_path
cost = d{1}(1)
Ptr = prod(P_tr(best_path))

figure
hold on
for i=1:length(best_path)-1  %plot path
    plot(coords(best_path(i),1), coords(best_path(i),2), 'r*')
    plot([coords(best_path(i),1), coords(best_path(i+1),1)],[coords(best_path(i),2), coords(best_path(i+1),2)], 'r-', 'LineWidth', 4)    
end
axis off
% axis equal