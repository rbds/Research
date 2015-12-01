clear
clc
close all

r(1,:) = linspace( 10, 20);
r(2,:) = linspace( 5, 10);

[time, state] = movement_simulation(r);
plot(time, state)
