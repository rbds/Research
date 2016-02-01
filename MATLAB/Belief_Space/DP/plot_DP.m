function [ ] = plot_DP(V, adj, P_tr, d, coords, n )
%plot_DP Plots results of DP path planning algorithm
%   Run multiple_objectives_DP_v2 first. Then, run this function, 
%       passing in the appropriate parameters (same names). n here is the 
%       node to plot to. n=1 is upper left, n=V is lower right.

figure
%create coordinates
% coords = [1 sqrt(V)];
% for i = 1:V-1
%    coords(i+1,:) = [floor(i/sqrt(V))+1, sqrt(V) - mod(i,sqrt(V))];
% end

m = max(max(adj));
hold on
for i=1:V
    for j=1:V
        if adj(i, j)>0
            red = adj(i,j)/m;
            blue = (P_tr(i) + P_tr(j))/2;
            plot([coords(i,1), coords(j, 1)], [coords(i,2), coords(j,2)], 'color', [0, 0, blue])
            plot(coords(j,1), coords(j,2), '*')
        end
    end
end
axis off
axis equal

t = n;
path = [n];
while (t<V-1)
    [~, ind] = min(d{t}(:,1));    %find minimum cost path at node t
    path(end+1) = d{t}(ind,3);
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

end

