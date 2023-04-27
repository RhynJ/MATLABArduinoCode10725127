% calculate Ardiuino simulation of inverted pendulum
% SFC of theta, thetaDot and position
% Lueberger observer only used to estimate theta, thetaDot
% estimate position made by directly integrating the velocity control signal
% with integral action on position
% nonlinear plant simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 23/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

% use default parameters for demo program
wantDefault = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get parameters for rod pendulum

% complete this function

params = GetRodPendulumParams(wantDefault, 5);
params

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get state space coefficients

% complete this function

c = GetStateSpaceCoesffs(wantDefault, params);
c

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get state space model with thetaDot, theta

% complete this function

ssm = GetSSModel2x2V(wantDefault, params);
disp('ssm.A')
disp(ssm.A)
disp(' ')
disp('ssm.B')
disp(ssm.B)
disp(' ')
disp('ssm.C')
disp(ssm.C)
disp(' ')
disp('ssm.D')
disp(ssm.D)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get state space model with thetaDot, theta and  position of cart

% complete this function

% integral action on position
ssmP = GetSSModel4x4V(params);
disp('ssmP.A')
disp(ssmP.A)
disp(' ')
disp('ssmP.B')
disp(ssmP.B)
disp(' ')
disp('ssmP.C')
disp(ssmP.C)
disp(' ')
disp('ssmP.D')
disp(ssmP.D)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build observer just for angle and angular velocity states
% calculate observer gain L here

% add code here
% .....

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for state space model with thetaDot, theta and  position of cart
% calculate SFC gain K here

% add code here
% .....

