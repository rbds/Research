%Belief Space Naviation
clear
close

x_start = [3;3; 0];
x_goal = [50; 50; 0];        %define goal region as a function of x, y position.
J = @(x,y) 128 - sqrt(x^2 + y^2);     %define cost function.
% always follow calculation of J with J = max(J, 0);
eps = 0.1;
max_iters = 100;        %number of iterations to run
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
E = {};                 %list of edges between nodes. define by a list of intermediate points?
Q = [];                 %priority queue
%create obstacles - need to turn this into a function, create more complex
%obstacles.
obstacles(:,1) = linspace(12, 25, 25);
obstacles(:,2) = linspace(10, 45, 25);

%state space motion model
A = 1;
B = 1;
C = 1;
% x = [x_pos, y_pos, theta]'
% x(t) = x(t-1) + u(t-1)+w(t)
% z(t) = x(t) + v(t)
% where w is the process noise and v is the system noise, both assumed to be gaussian.


%plot a single line obstacle
plot([obstacles(1,1);obstacles(end,1)],[obstacles(1,2);obstacles(end,2)]);
hold on
plot(V{1,1}.x(1),V{1,1}.x(2), 'gx')
plot(x_goal(1), x_goal(2), 'rx')
axis([0 60 0 60])
while count <max_iters
   %sample a point
   x = rand(2,1); 
   x_new(1,1) = x(1,1)*(course(3) - course(1)) + course(1);
   x_new(2,1) = x(2,1)*(course(4) - course(2)) + course(2);
   x_new(3,1) = 0;
   
   %plot new point
   plot(x_new(1),x_new(2),'kx');
   %find nearest neighbor
    for i=1:(length(V))                            %iterates through each valid node 
      distance = norm(V{i,1}.x -  x_new);                  %finds length between vertex and new coordinate
        if (i==1) || (distance < mindist)               %1st iteration or distance is less than minimum dist
            mindist = distance;
            imin = i;
            v_nearest = V{i,1}.x;                           %chooses single closest point. Consider changing this to a k-nearest search
        end 
    end
    edge = connect(v_nearest, x_new); 
   %draw ellipses, propogate
   [success, x_ret] = propogate(edge, V{imin, 1}, x_new, chance_constraint, obstacles);
   if success == 1 %if there is a connection
        %add x_new to v
        %calculate properties of x_new (like cost, SOC)
            v_new = x_ret;  
            v_new.cost = norm(v_new.x - v_nearest);
            v_new.parent = imin;
%             v_new.SOC = v_new.cost*drain
            %need to write a length function.
%             V{end,1}.cost = len(E{end+1});
            V{end+1, 1} = v_new; %#ok<SAGROW>
        %add edge 
            E{end +1} = edge;    %#ok<SAGROW>
        %add x_new to queue
            Q(end+1) = length(V);
            Q = order(Q);
        %find all v within a radius
            c=10; %coefficient for radius of the ball
            num_nodes = size(V,1)*size(V,2);
            rad = c*(log(num_nodes)/num_nodes)^(1/2);
            %find all vertices within radius
            v_near = [];        %v_near stores the index of the vertex in V.
            for i=1:length(V) - 1;
                dist = norm(V{i,1}.x - v_new.x);
                if dist < rad
                   v_near(end+1) = i;  %#ok<SAGROW>
                end
            end
        %for all nearby vertices
        for k=1:length(v_near)
            %add edges
            e = connect(V{v_near(k),1}.x, V{end,1}.x);
            E{end + 1} = e;
            %update queue
            Q(end+1) = v_near(k);
        end
        %sort Q
        %while queue is not empty
        Q = order(Q);
        while (~isempty(Q)) 
            %n = pop from queue
            [n,Q] = pop(Q);
            for k=1:length(v_near) %for all vneighbor of v(n)
                e_near = connect(V{v_near(k)}.x, V{n}.x);
                %n = PROPOGATE(e_near, n)
                n_new = propogate(e_near, V{v_near(k)}, V{n}, chance_constraint, obstacles);
                %if APPENDBELIEF(v_near, n)
                    %add n to queue
                %end if
            
            end
        end
    end
   count = count +1;
end

mindist = 100;
for i=1:length(V)
   d = norm(V{i,1}.x - x_goal);
   if d<mindist
      mindist = d;
      i_goal= i;
   end
end
plot(V{i_goal,1}.x(1), V{i_goal,1}.x(2), 'ro')
V{i_goal,1}.x
par = V{i_goal, find(V{i_goal}.cost == min(V{i_goal}.cost))}.parent;
i=i_goal;
while par>1
    plot([V{i}.x(1);V{par}.x(1)],[V{i}.x(2);V{par}.x(2)], 'g')
    i = par;
    par = V{par,1}.parent;
end
plot([V{i}.x(1); x_start(1)], [V{i}.x(2); x_start(2)], 'g');

a=V{i_goal,1}.sigma(1); %horizontal radius
b=V{i_goal,1}.sigma(2); %vertical radius
x0=V{i_goal,1}.x(1); % x0,y0 ellipse centre coordinates
y0=V{i_goal,1}.x(2);
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
plot(x,y,'b')
