%draw map
clear
close all

% ha = axes('units','normalized','position',[0 0 1.1 1]);
% 
% uistack(ha,'bottom');
% 
% I=imread('sample_map.png');
% hi = imagesc(I);
% colormap gray
% 
% set(ha,'handlevisibility','off','visible','off')
% 
% axes('position',[0 0 .99 .99])
% plot(rand(10))
costs = [.3 .2 .3 .5 .3 .3 .3 .3 .5 .4 .3 .4 .4 .3 .2;
        .3 .5 .5 .3 .2 .2 .2 .3 .7 .6 .3 .3 .5 01 .3;
        .2 .3 .1 .2 .4 .3 .2 .2 .7 .8 .3 .3 .4 .3 .3;
        .3 .3 .2 .1 .1 .2 .3 .2 .4 .4 .3 .6 .3 .2 .2;
        .3 .3 .5 .4 .2 .1 .1 .2 .5 .2 .8 01 .7 .6 .4;
        .3 .5 .7 .7 .3 .2 .2 .1 .1 .2 .8 01 01 01 .5;
        .3 .3 .5 .4 .3 .3 .2 .2 .1 .1 .2 .5 .8 .9 .3;
        .3 .3 .3 .3 .3 .3 .3 .2 .2 .1 .2 .2 .6 01 .4;
        .3 .3 .3 .2 .4 .3 .3 .3 .2 .1 .2 .3 .4 .6 .3;
        .3 .3 .3 .2 .5 .3 .2 .2 .1 .2 .2 .3 .3 .3 .3;
        .2 .1 .2 .2 .2 .1 .1 .1 .5 .5 .3 .3 .3 .3 .3;
        .2 .2 .1 .1 .1 .2 .2 .2 .5 .3 .3 .3 .3 .4 .3;
        .1 .1 .1 .1 .1 .2 .2 .5 .5 .3 .3 .6 .6 .4 .3;
        .1 1 .1 .1 .1 .2 .2 .2 .3 .3 .3 .6 .6 .4 .3;
        .2 .3 .3 .3 .3 .3 .3 .3 .3 .3 .3 .6 .6 .4 .3];
    
% costs = flipud(costs);
costs = costs.^2;
    
P_tr =sqrt([.90 .50 .50 .75 .99 .99 .99 .99 .99 .99 .99 .99 .99 .80 .99;
       .90 .50 .50 .99 .99 .99 .99 .99 .99 .80 .60 .75 .95 .90 .99;
       .90 .80 .45 .75 .99 .99 .99 .90 .99 .90 .50 .50 .95 .99 .99;
       .99 .99 .90 .50 .50 .75 .99 .99 .99 .99 .70 .60 .90 .99 .99;
       .99 .95 .90 .85 .75 .50 .60 .80 .75 .99 .99 .99 .95 .95 .99;
       .99 .90 .90 .25 .99 .95 .75 .50 .60 .99 .99 .99 .99 .99 .99;
       .99 .99 .90 .90 .99 .99 .99 .90 .50 .65 .99 .75 .99 .99 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .90 .50 .60 .99 .75 .99 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .90 .50 .90 .99 .99 .99 .99;
       .99 .99 .99 .99 .75 .99 .99 .99 .60 .60 .90 .99 .99 .99 .99;
       .50 .50 .60 .60 .60 .60 .50 .50 .65 .75 .99 .99 .99 .99 .99;
       .60 .60 .40 .50 .50 .50 .60 .50 .99 .99 .99 .99 .99 .80 .99;
       .60 .60 .40 .50 .50 .50 .60 .50 .99 .99 .99 .99 .99 .80 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .85 .99;
       .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .99 .85 .99]);
% P_tr = flipud(P_tr);   
axis off
axis equal
    

V = 195;
s = 13;
N = V^2;
i_vals = [];
j_vals = [];

for i = 1:V    %for each row in adjacency matrix
   if (mod(i,s) >0) %if it isn't on the right edge of grid 
    j_vals(end+1) = i; %add node to right
    i_vals(end+1) = i+1;
   end

   if (mod(i, s) ~=1) %if node isn't on the left edge of grid
    j_vals(end+1) = i; %add node to left
    i_vals(end+1) = i-1;
   end
   
   if (i> s) %if vertex isn't on top of grid
    j_vals(end+1) = i ;   %add next node up
    i_vals(end+1) = i - s;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1 - s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1 -s;
       end
   end
   
   if (i<=(V- s))   %if vertex isn't on bottom of grid
    j_vals(end+1) = i ;  %add next node down
    i_vals(end+1) = i+s ;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1+ s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1+s;
       end
   end   
end



% %create coordinates
coords = [];
for i = 1:N
   block_no = ceil(i/V);
   V_no = i - (block_no-1)*V;

   row = mod(V_no,s);
   col = ceil(V_no/s);
      if (row==0) row = s; end 
   coords(end+1,:) = [row, col]; 
end

%create adjacency matrix
% vals = 10*abs(randn(length(i_vals), 1));  %generate random costs
vals = zeros(size(i_vals));
for i=1:length(i_vals)
    vals(i) = mean([costs(i_vals(i)), costs(j_vals(i))]);
end
a= sparse(i_vals, j_vals, vals); %one section of the adjacency matrix
adj = sparse(V^2, V^2);
for i=1:V-1 %assemble adjacency matrix from sections
   adj((i-1)*V+1:(i-1)*V+V, i*V+1:i*V+V) = a;
end
%  spy(adj)
% gplot(adj, coords, '*-') %plot graph

axis off
axis equal



figure
for i = 1:V
    v = [ coords(i,2)-0.5 coords(i,1)-0.5; coords(i,2)-0.5 coords(i,1)+0.5; coords(i,2)+0.5 coords(i,1)+0.5; coords(i,2)+0.5  coords(i,1)-0.5 ];
    patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [0 0 1], 'FaceAlpha', costs(i).^.2)
    
end

axis off
axis equal
axis([0 16 0 15])
title('Cost Map')

blues = linspace(1, 0, 64);
newmap = [zeros(64,1) zeros(64,1) blues'];
% alpha(0.5)
colormap(newmap);   %activate it
colorbar

% 
% figure
% for i = 1:V
%     v = [ coords(i,2)-0.5 coords(i,1)-0.5; coords(i,2)-0.5 coords(i,1)+0.5; coords(i,2)+0.5 coords(i,1)+0.5; coords(i,2)+0.5  coords(i,1)-0.5 ];
%     patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [1 0 0 ], 'FaceAlpha', 1 - P_tr(i))
% end
% 
% 
% axis off
% axis equal
% axis([0 16 0 15])
% title('Map of P_{tr}(x)')
% 
% reds = linspace(1, .5, 64);
% newmap2 = [reds' zeros(64,1) zeros(64,1)];
% colormap(newmap2);   %activate it
% colorbar

