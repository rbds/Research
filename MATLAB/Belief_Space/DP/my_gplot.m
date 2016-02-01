figure
%create coordinates
coords = [1 sqrt(V)];
for i = 1:V-1
   coords(i+1,:) = [floor(i/sqrt(V))+1, sqrt(V) - mod(i,sqrt(V))];
end

m = max(max(adj));
hold on
for i=1:V
    for j=1:V
        if adj(i, j)>0
            red = adj(i,j)/m;
            blue = (P_tr(i) + P_tr(j))/2;
            plot([coords(i,1), coords(j, 1)], [coords(i,2), coords(j,2)], 'color', [red, 0, blue])
            plot(coords(j,1), coords(j,2), '*')
        end
%         val = num2str(costs(i));
%         text(coords(i,1), coords(i,2), val)
    end
end
axis off
axis equal