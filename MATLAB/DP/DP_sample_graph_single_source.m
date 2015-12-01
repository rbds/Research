%DPM on Sample Graph
%Single Source tree.
clear
close all
coords = [1,1; 3,4.5; 4,0; 8,4; 8,1.5; 10,0; 12,4; 13,1; 16,3];
V = 9; %number of vertices
E = [1,2; 1,3; 2,3; 3,4; 3,5; 4,5; 4,6; 4,7; 5,6; 6,7; 7,8; 7,9; 8,9];  
adj = zeros(V);
d = zeros(V,1);
par_point = zeros(V,1);
%build adjacency matrix
for i=1:V
    for j=1:length(E)
       if E(j,1) == i
          adj(i, E(j,2)) = abs(10*randn(1)); 
       end
    end
end

gplot(adj, coords, '*-')


for i = 2:V
    parents = find(adj(:,i) ~=0);
    w = adj(parents, i);
    [d(i), k] = min( w + d(parents));
    par_point(i) = parents(k);
end
t = V;
path = [];
while (t>0)
    path(end+1) = par_point(t);
    t = path(end);
end
path
