clear
clc
close all

hold on
grid on

sides = 100; %make this a multiple of 5
axis([0 sides 0 sides])
n = sides^2;
G = zeros(n,n);
map = round(0.6*rand(sides,sides));
map(1,1) = 0;
map(end,end) = 0;
map_to_show = (flipud(map'));
%draw map
for i=1:sides
    for j=1:sides
        if map_to_show(i,j)==1
           rectangle('Position', [i-1, j-1, 1, 1], 'FaceColor', 'k')
        end
    end
end
rectangle('Position', [sides-1,0,1,1], 'FaceColor', 'g')
rectangle('Position', [0,sides-1,1,1], 'FaceColor', 'r')
map = fliplr(flipud(map)); % get map to match image;
%generate adjacency matrix
for i=1:sides
    for j=1:sides
        if map(i,j)==1
            continue
        end
        if i+1<=sides
            if map(i+1,j)==0
                G((j-1)*sides+i,(j-1)*sides+i+1)=1;
                G((j-1)*sides+i+1,(j-1)*sides+i)=1;

            end
        end
        if j+1<=sides
            if map(i,j+1)==0
                G((j-1)*sides+i,(j-1)*sides+i+sides)=1;
                G((j-1)*sides+i+sides,(j-1)*sides+i)=1;
            end
        end
%         if i+1<=sides && j+1<=sides
%             if map(i+1,j+1)==0
%                 G((j-1)*sides+i, (j-1)*sides+i+sides+1)=sqrt(2);
%                 G((j-1)*sides+i+sides+1, (j-1)*sides+i)=sqrt(2);
%             end
%         end
    end
end
% % %Generate gray map
% gray_map_a = zeros(sides/5,sides/5);
% % 
% for i=1:sides
% %    
%    zy = 1;
%    for j = 1:5:sides
%       gray_map_a(i,zy) = sum(map(i,j:j+4)); 
%       zy = zy+1;
%    end
% end
% zx = 0;
% for i=1:sides/5
%  zx = zx + 1;   
%    for j = 1:size(gray_map_a,2)
%        gray_map(zx,j) = sum(gray_map_a(i:i+4,j)) ;
%    end
% end
% 
% figure
% for i = 1:(sides/5) %positions are incorrect
%     for j = 1:(sides/5)
%      rectangle('Position', [i-1, j-1, 1, 1], 'FaceColor',  [gray_map(i+j)/sides gray_map(i+j)/sides gray_map(i+j)/sides]);
%     end
% end
% figure(1)
% hold on

[path, cost] = dijkstra2(n, G, 1, n)  % number of sides, adjacency, start, end
%figure
%convert path back to x,y
path_xy = [0 0];
for i=1:length(path)
    ind = path(i);
    x=0;
    y=0;
    while(ind>sides)
        ind = ind-sides;
        y = y+1;
    end
        x = ind;
        y = y+1; %necessary to make the x,y coords symmetrical
   path_xy =  [path_xy; x y];
end
path_xy(1,:) = []

px = sides+1 - path_xy(:,1);
path_xy(:,1) = px;
path_xy = path_xy - .5;
plot(path_xy(:,2),path_xy(:,1), 'b', 'LineWidth', 3)

% ind_i = 1;
% ind_j = 1;
% for i = 1:5:sides
%     for j = 1:5:sides
%         map_zoomed(ind_i,ind_j) = mean(mean(map(j:j+4,i:i+4)));
%         ind_j = ind_j +1;
%     end
% ind_j =1;
% ind_i = ind_i+1;
% end