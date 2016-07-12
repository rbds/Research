
xdata =[0.5000
    1.0000
    1.5000
    2.0000
    2.5000
    3.0000
    3.5000
    4.0000
    4.5000
    5.0000
    6.0000
    7.0000
    8.0000];
ydata =[1.0141
    1.0175
    1.0243
    1.0305
    1.0363
    1.0470
    1.0549
    1.0666
    1.0774
    1.0971
    1.1213
    1.1953
    1.2617];
%data averaged over 25 runs.
figure
plot(xdata, ydata, 'b*', 'LineWidth', 5)
hold on
dat  = [xdata, xdata.*ydata, ydata, ones(length(xdata),1)]; %hyperbolic fit
[U,S,V] = svd(dat);
fit = V(:,end);
xfill = linspace(0,10,101);
line = (-fit(1)*xfill - fit(4))./(fit(2)*xfill + fit(3));
axis([0 10 1 1.5])
plot(xfill, line, 'r', 'LineWidth', 2)
title('Path Length vs. Obstacle Density', 'fontsize', 18)
set(gca,'fontsize', 14)
xlabel('Obstacle Density (%)')
set(gca,'fontsize', 14)
ylabel('Ratio of path length to straight line')
set(gca,'fontsize', 14)
legend('numerical simulation', 'exponential curve fit', 'location', 'NorthWest')
set(gca,'fontsize', 14)
