%RRT Path Length Plots
close
opt = [42.940 42.940 42.940 42.940];
its = [100 250 500 1000];
length = [49.079 45.308 44.267 43.568];

plot(its, length, 'o-', its, opt)
title('RRT* Average Path Legnth vs. Number of Iterations')
xlabel('Iterations')
ylabel('Path Length (dimensionless)')
legend('Calculated Path Length')
legend('Calculated Path Length','Optimal Path')



%% with sun
close all
opt = repmat(55.0821, 1, 4);
its = [100 250 500 1000];
time = [ 1.13 6.01 24.77 89.93];
length = [59.722  57.71 57.03 55.42];
val = length./opt;
figure
[hax, hline1, hline2] = plotyy(its, time, its, val);
title('BE-RRT Running Time and Path Length vs. Number of Iterations')
xlabel('Iterations')
ylabel(hax(1), 'Running Time [seconds]')
ylabel(hax(2), 'Ratio of Path Length')
legend('Average Running Time', 'Ratio of Path Length/Optimal Path')

