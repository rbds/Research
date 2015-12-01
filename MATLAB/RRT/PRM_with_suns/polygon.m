close
L= linspace(0,2.*pi,6);
xv = cos(L);
yv = sin(L);

rng default
xq = randn(250,1);
yq = randn(250,1);
in = inpolygon(xq,yq,xv,yv);

plot(xv,yv)
axis equal
hold on
plot(xq(in),yq(in),'r+')
plot(xq(~in),yq(~in), 'bo')