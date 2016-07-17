clear
clc
close all

% hold on
% grid on

sides = 100; %make this a multiple of 5
axis([0 sides 0 sides])
N = [50, 100,  150, 200, 250, 300, 350, 400, 450, 500, 600, 700, 800, 900, 1000, 2500, 3000, 4000, 5000];
% N = [500, 750, 1000];
D = linspace(5,35, 12);
% N = 750;
% D = 25;
% G = zeros(sides^2,sides^2);
n_fills = length(N);
ni = 1;

path_lengths = zeros(n_fills, length(D), ni);
num_legs = zeros(n_fills, length(D),ni);

% course = [0 0 sqrt(env) sqrt(env)];

for kk = 1:ni
    for ii = 1:n_fills
        for jj = 1:length(D)
            
        pts= [sides/2 + D(jj)*randn(1,N(ii)); sides/2 + D(jj)*randn(1,N(ii))];
        n = hist3(pts', [sides, sides]);
        map = n>0;
        map(1,1) = 0;
        map(end,end) = 0;
%         map_to_show = (flipud(map'));
%         % draw map
%         clf
%         hold on
%         for i=1:sides
%             for j=1:sides
%                 if map_to_show(i,j)==1
%                    rectangle('Position', [i-1, j-1, 1, 1], 'FaceColor', 'k')
%                 end
%             end
%         end
%         rectangle('Position', [sides-1,0,1,1], 'FaceColor', 'g')
%         rectangle('Position', [0,sides-1,1,1], 'FaceColor', 'r')

        map = rot90(map,2); % get map to match image;
        %generate adjacency matrix
        G = zeros(sides^2,sides^2);
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
                if i+1<=sides && j+1<=sides
                    if map(i+1,j+1)==0
                        G((j-1)*sides+i, (j-1)*sides+i+sides+1)=sqrt(2);
                        G((j-1)*sides+i+sides+1, (j-1)*sides+i)=sqrt(2);
                    end
                end
            end
        end


        [path, ~] = dijkstra2(sides^2, G, 1, sides^2);  % number of sides, adjacency, start, end
        % figure
        % convert path back to x,y
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
        path_xy(1,:) = [];

        px = sides+1 - path_xy(:,1);
        path_xy(:,1) = px;
        path_xy = path_xy - .5;
%         plot(path_xy(:,2),path_xy(:,1), 'b', 'LineWidth', 3)

        cost = 0;
        for zz= 1:length(path_xy)-1
            cost = cost + norm(path_xy(zz,:) - path_xy(zz+1,:));
        end
        
        path_lengths(ii, jj, kk) = cost/norm(path_xy(1,:) - path_xy(end,:));
        num_legs(ii, jj, kk) = length(path);
        cost
%         drawnow
        end
    end
end
