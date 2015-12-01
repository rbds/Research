%DPM on Sample Graph - bottom up
%Single Target
%output is the optimal path length from each node to end.
clear
close all
coords = [1,1; 3,5; 4,0; 8,4; 8,1.5; 10,0; 12,4; 13,1; 16,3];
V = 9; %number of vertices
E = [1,2; 1,3; 2,3; 3,4; 3,5; 4,5; 4,6; 4,7; 5,6; 6,7; 7,8; 7,9; 8,9];  
adj = zeros(V);
d = zeros(1,V);
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
% wgPlot(adj, coords)

for i=V-1:-1:1
    conns = find(adj(i,:)~=0);
    [d(i), ind] = min( d(conns) + adj(i, conns));
    par_point(i) = conns(ind);
end

t = 1;
path = [1];
while (t<V)
    path(end+1) = par_point(t);
    t = path(end);
end
path
hold on
for i=1:length(path)-1
    plot(coords(path(i),1), coords(path(i),2), 'r*')
    plot([coords(path(i),1), coords(path(i+1),1)],[coords(path(i),2), coords(path(i+1),2)], 'r-')    
end
axis off