%Belief Space Navigation
clear
% close
clf
x_start = [3; 3];
x_goal = [50; 50];        %define goal region as a function of x, y position.
opt_dist = norm(x_start - x_goal);
max_iters = 500;        %number of iterations to run
count=1;
course = [0 0 50 50];     %course size - rectangular (x1 y1 x2 y2)
 chance_constraint = .05;   
% initialize vertices, nodes
V = {};             %cell array, each row is a vertex, and each element in the row is a node
n(1).x = x_start;       %node 1 is at x_start
n(1).parent = [1, 1];   %parent node is given by index in cell array v.
n(1).sigma = [.01; .01]; 

n(1).lambda = 0 + .01*randn(3,1);
n(1).SOC = 1;
n(1).cost = 0;
V{1,1} = n(1);
n = [];
n_lowest = 1;
%E = {};                 %list of edges between nodes. define by a list of intermediate points?
Q = [1 1];
%create obstacles - need to turn this into a function, create more complex
%obstacles.
obstacles(:,1) = linspace(20, 30, 250);
obstacles(:,2) = linspace(20, 51, 250);

%state space motion model
% x = [x_pos, y_pos, theta]'
% x(t) = x(t-1) + u(t-1)+w(t)
% z(t) = x(t) + v(t)
% where w is the process noise and v is the system noise, both assumed to be gaussian.


%plot a single line obstacle
plot([obstacles(1,1);obstacles(end,1)],[obstacles(1,2);obstacles(end,2)]);
hold on
plot(V{1,1}.x(1),V{1,1}.x(2), 'gx')
plot(x_goal(1), x_goal(2), 'rx')
axis([0 course(3)+5 0 course(4)+5])
while count <max_iters      
   %sample a point
   x = rand(2,1); 
   x_new(1,1) = x(1,1)*(course(3) - course(1)) + course(1);
   x_new(2,1) = x(2,1)*(course(4) - course(2)) + course(2);
      
   %plot new point
   plot(x_new(1),x_new(2),'kx');
   %find nearest neighbor
    for i=1:(size(V,1))                            %iterates through each valid node 
      distance = norm(V{i,1}.x -  x_new);                  %finds length between vertex and new coordinate
        if (i==1) || (distance < mindist)               %1st iteration or distance is less than minimum dist
            mindist = distance;
            imin(1) = i;
        end 
    end
    
    %parent should be the node at that vertex with the lowest cost
     %    %cost function here is simply covariance (ignoring path length cost)
    smallest_cost =sqrt(V{imin(1),1}.sigma(1)^2 + V{imin(1),1}.sigma(2)^2); 
    empt = cellfun(@isempty, V);
    temp = empt(imin(1),:).*(1:size(empt,2));
    vals = temp(temp==0);
    imin(2) = 1;    
    for i = 2:length(vals)
           cost = V{imin(1), i}.cost;
        if cost< smallest_cost
           smallest_cost = cost;
           imin(2) = i;
        end
    end
    v_nearest = V{imin(1),imin(2)}.x;                           %chooses single closest point. Consider changing this to a k-nearest search
    [edge] = connect(v_nearest, x_new);
   %draw ellipses, propagate
   [success, x_ret] = propogate(edge, V{imin(1), imin(2)}, x_new,  chance_constraint, obstacles);
   if success == 1 %if there is a connection
        %add x_new to v
        %calculate properties of x_new (like cost, SOC)
            v_new = x_ret;  
            v_new.cost = norm(v_new.x - v_nearest) + V{imin(1),imin(2)}.cost;
            v_new.parent = [imin(1), imin(2)];       %adjust to find correct parent node, maybe with least covariance
%             v_new.SOC = v_new.cost*drain

            V{end+1, 1} = v_new; %#ok<SAGROW>
        %add edge 
           % E{end +1} = edge;    %#ok<SAGROW>
        %add x_new to queue
            Q= [size(V,1), 1];
            
%         %find all v within a radius
            c=2/pi*(1+1/2)^(1/2)*(course(3)*course(4)/pi)^(1/2); %coefficient for radius of the ball
            num_nodes = size(V,1)*size(V,2);
            rad = c*(log(num_nodes)/num_nodes)^(1/2);
            %find all vertices within radius
            v_near = [];        %v_near stores the index of the vertex in V.
            for i=1:size(V,1) - 1;          %Try nearest neighbors here. 
                dist = norm(V{i,1}.x - v_new.x);
                if dist < rad
                   v_near(end+1) = i;  %#ok<SAGROW>
                end
            end
     %connect to k-nearest neighbors (naive search, try using k-d tree
%      k_neighbors = 3; % number of nearest neighbors to use. 
%                list = zeros(size(V,1)-1,1);
%             for i=1:size(V,1)-1
%                list(i) = norm(V{end,1}.x - V{i,1}.x);
%             end
%             [d, neighbors] = sort(list);
%             if (length(neighbors)>k_neighbors)
%                 v_near = neighbors(1:k_neighbors); 
%             else
%                 v_near = neighbors;
%             end

            %for all nearby vertices
        for k=1:length(v_near)
            %add edges
            [e] = connect(V{v_near(k),1}.x, V{end,1}.x);
           % E{end + 1} = e;
                empt = cellfun(@isempty, V);
                temp = empt(v_near(k),:).*(1:size(empt,2));
                vals = temp(temp~=0);
                if isempty(vals)
                        Q(end+1, :) = [v_near(k), 1]; %#ok<SAGROW>
                else
                    for ind = 1:(vals(1) - 1)    
                        Q(end+1, :) = [v_near(k), ind]; %#ok<SAGROW>
                    end
                end

        end
       %while queue is not empty
       % Q = order(Q);
        while (~isempty(Q)) 
            %n = pop from queue
            [n,Q] = pop(Q);
            for k=1:length(v_near) %for all vneighbor of v(n)
                [e_near] = connect(V{v_near(k),1}.x, V{n(1),n(2)}.x);
                %n = PROPAGATE(e_near, n)
                [success, n_new] = propogate(e_near, V{v_near(k),1}, V{n(1),n(2)}.x, chance_constraint, obstacles);
                if (success) %if node can successfully be propagated
                          least_cost = 1000;
                          for i = 1:(size(V,2)) %find the node in v_nearest with the lowest cost 
                            if (isfield(V{v_near(k), i},'cost'))
                              cost = V{v_near(k),i}.cost;
                                if   cost < least_cost || i==1
                                  least_cost = V{v_near(k),i}.cost;
                                  if n(1) ~= n_lowest
                                    n_lowest = i;
                                  end 
                                end
                            end
                          end
                    n_new.cost = V{v_near(k), n_lowest}.cost + norm(V{v_near(k), 1}.x - n_new.x);
                 %if APPENDBELIEF(v_near, n)
                    %add n to queue
                 %end if
%                   
                 [V, success] = AppendBelief(V, n_new, n);
                 if (success)
                    empt = cellfun(@isempty, V);
%                     temp = empt(v_near(k),:).*(1:size(empt,2));
%                     vals = temp(temp~=0); 
%                     Q(end+1,:) =  [v_near(k) vals(1)-1]; %add n back to the queue

                    temp = empt(n(1),:).*(1:size(empt,2)); %add in properties for new node
                    vals = temp(temp~=0);        
                        if (isempty(vals))
                            V{n(1), end+1} = n_new; %#ok<SAGROW>
                            V{n(1), end}.parent = [v_near(k),n_lowest];
%                             Q(end+1,:) = [n(1) size(V,2)];
                        else
                            V{n(1), vals(1)} = n_new;
                            V{n(1), vals(1)}.parent = [v_near(k), n_lowest]; % node with the lowest cost
%                             Q(end+1, :) = [n(1) vals(1)];
                        end
                 end 
                end
                

            end
        end
    end
   count = count +1;
end

mindist = 100; %% find vertex closest to the goal
for i=1:length(V)
   d = norm(V{i,1}.x - x_goal);
   if d<mindist
      mindist = d;
      i_goal= [i, 1];
   end
end
empt = cellfun(@isempty, V);
row = empt(i_goal(1),:);
vals = row(row==0);
lowest_cost = 1000; % find node with least cost.
for ind = 1:length(vals)  
 temp =  V{i_goal(1),ind}.cost;
 if (temp<lowest_cost)
    lowest_cost = temp;
    i_goal(2) = ind;
 end
end


% plot(V{i_goal(1),i_goal(2)}.x(1), V{i_goal(1),i_goal(2)}.x(2), 'ro')
V{i_goal(1),i_goal(2)}.x %print location of goal node
par = V{i_goal(1),i_goal(2)}.parent; 

path = par;
i=i_goal;

while par(1)>1      %plot path
    plot([V{i(1), i(2)}.x(1);V{par(1),par(2)}.x(1)],[V{i(1), i(2)}.x(2);V{par(1),par(2)}.x(2)], 'g') %should be fixed to V{par(1),V{par(2)}

        x1 = V{i(1),i(2)}.x(1) + V{i(1),i(2)}.sigma(1);%*cos(V{i(1),i(2)}.x(3));
        x2 = V{i(1),i(2)}.x(1) - V{i(1),i(2)}.sigma(1);%*cos(V{i(1),i(2)}.x(3));
        y1 = V{i(1),i(2)}.x(2) + V{i(1),i(2)}.sigma(1);%*sin(V{i(1),i(2)}.x(3));
        y2 = V{i(1),i(2)}.x(2) - V{i(1),i(2)}.sigma(1);%*sin(V{i(1),i(2)}.x(3));
        ecc = sqrt(abs(V{i(1),i(2)}.sigma(1)^2) - abs(V{i(1),i(2)}.sigma(2)^2));
         a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
         b = a*sqrt(abs(1-ecc^2));
         t = linspace(0,2*pi);
         X = a*cos(t);
         Y = b*sin(t);
         w = atan2(y2-y1,x2-x1);
         x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
         y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
         plot(x,y,'r-')
 
 i = par;
par = V{par(1),par(2)}.parent;
path(end+1,:) = par; %#ok<SAGROW>
end
        x1 = V{i(1),i(2)}.x(1) + V{i(1),i(2)}.sigma(1);%*cos(V{i(1),i(2)}.x(3));
        x2 = V{i(1),i(2)}.x(1) - V{i(1),i(2)}.sigma(1);%*cos(V{i(1),i(2)}.x(3));
        y1 = V{i(1),i(2)}.x(2) + V{i(1),i(2)}.sigma(1);%*sin(V{i(1),i(2)}.x(3));
        y2 = V{i(1),i(2)}.x(2) - V{i(1),i(2)}.sigma(1);%*sin(V{i(1),i(2)}.x(3));
        ecc = sqrt(abs(V{i(1),i(2)}.sigma(1)^2) - abs(V{i(1),i(2)}.sigma(2)^2));
         a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
         b = a*sqrt(abs(1-ecc^2));
         t = linspace(0,2*pi);
         X = a*cos(t);
         Y = b*sin(t);
         w = atan2(y2-y1,x2-x1);
         x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
         y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
         plot(x,y,'r-')
plot([V{i(1),1}.x(1); x_start(1)], [V{i(1),1}.x(2); x_start(2)], 'g');

path %#ok<NOPTS> %print out path
%draw ellipse around point closest to the goal.


% %plot the tree
% for i=1:size(V,1)
%     for j=1:size(V,2)
%     plot([V{i,1}.x(1), V{V{i,1}.parent(1),1}.x(1)], [V{i,1}.x(2), V{V{i,1}.parent(2),1}.x(2)], 'k:')
% 
%     end
% end