close all
clear all
clc

% use default parameters for demo program
wantDefault = 0;

% get parameters for rod pendulum
params = GetRodPendulumParams(wantDefault, 5);
params


% get state space coefficients
c = GetStateSpaceCoesffs(wantDefault, params);
c

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get state space model with thetaDot, theta
% complete this function

ssm = GetSSModel2x2V(params, c);
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
ssmP = GetSSModel4x4V(params,c);
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

