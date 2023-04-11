clc;clear all;

[t,x] = ode45(@stateSpaceModel,[0 10], [pi/2 0 pi/4] ); %the file i want to test, how long for and the initial conditions     


plot(t,x)
