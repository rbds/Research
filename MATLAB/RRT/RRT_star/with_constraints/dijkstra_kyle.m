function [path, cost] = dijkstra(start, goal, adj)
% [path, cost] = dijkstra(start, goal, graph)
% Author: Kyle Crandall
% Description performs a dijkstra search on a graph to find the cheapest
% solution to navigate between start and the goal.
%
% Inputs:
%  start - index of the starting position in graph
%  goal  - index of the goal position in graph
%  adj   - adjacency matrix for graph
% Outputs:
%  path - array of indecies to follow for optimal path
%  cost - cost of ideal path

	graph.adj = adj;
	graph.locked = zeros(1, length(graph.adj));
	graph.cost = ones(1, length(graph.adj)) * inf;
	graph.cost(start) = 0;
	graph.back_ptr = zeros(1, length(graph.adj));
	
	while 1
		% find lowest cost unlocked node
		unlocked = find(~graph.locked);
		[cur_cost, cur] = min(graph.cost(unlocked));
		cur = unlocked(cur);
		
		% find list of nodes connected to current node
		connected = find(graph.adj(cur, :));
		
		% update costs for connected nodes
		for j = 1:length(connected)
			if graph.adj(cur, connected(j)) + cur_cost < graph.cost(connected(j))
				graph.cost(connected(j)) = graph.adj(cur, connected(j)) + cur_cost;
				graph.back_ptr(connected(j)) = cur;
			end
		end
		
		% lock current node
		graph.locked(cur) = 1;
		
		% if current node is the goal, we are done
		if (cur == goal)
			break;
		end
	end
	
	% pull path out of search
	
	path(1) = goal;
	
	while path(1) ~= start
		path = [graph.back_ptr(path(1)) path];
	end
	
	cost = graph.cost(goal);
end
	
	