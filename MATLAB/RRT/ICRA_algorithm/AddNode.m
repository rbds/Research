function Tree = AddNode(Tree,p,iPrev, cost_map)
Tree(end+1).p = p;
Tree(end).iPrev = iPrev;
x = (p(1)+2.5)/5;
y = (p(2)+2.5)/5;
Tree(end).place = y+(x-1)*7;
%cobs = alpha*exp(-d);
% mind2g = 

Tree(end).cost = Tree(iPrev).cost + norm(Tree(end).p - Tree(iPrev).p)*(1 + cost_map(Tree(end).place));

%Tree(iPrev).children(end + 1) = length(Tree);
end