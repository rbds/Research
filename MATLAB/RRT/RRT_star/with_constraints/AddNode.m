function Tree = AddNode(Tree,p,iPrev)
Tree(end+1).p = p;
Tree(end).iPrev = iPrev;

Tree(end).cost = Tree(iPrev).cost + norm(Tree(end).p - Tree(iPrev).p);

%Tree(iPrev).children(end + 1) = length(Tree);
end