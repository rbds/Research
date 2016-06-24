function [] =  circle(x,y,r,color)

hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit,color,'LineWidth',3);
drawnow;
%hold off

end