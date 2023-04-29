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

% get parameters for rod pendulum
params = GetRodPendulumParams(wantDefault, 5);


% get state space coefficients
c = GetStateSpaceCoesffs(wantDefault, params);

% get state space model with thetaDot, theta


% get state space model with thetaDot, theta and  position of cart
% integral action on position
ssmP = GetSSModel4x4V(c);
ssm = GetSSModel2x2V(wantDefault, params, c);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build observer just for angle and angular velocity states
% calculate observer gain L here

% put your code for calculating L here
PX = [-6, -7];
L = place(ssm.A, ssm.C', PX);
disp(' ')
disp('L')
disp(L)

% for state space model with thetaDot, theta and  position of cart
% calculate SFC gain K here

 KX = [-0.8, -2, -3, -  4];
 K = place(ssmP.A,ssmP.B, KX);
 disp(' ')
 disp('K')
 disp(K)
 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup time points
dt =  0.0100;
Tfinal = 5;
t = 0:dt:Tfinal;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titleMessage = 'NLSimulateVelSFCLArduino4x4 partial observer Arduino';
disp(titleMessage)

% you cann base thgis on the code in Main_RunDemoUncontrollerPendulumV.m
%   initialize arrays
xData=[];
yData=[];
tData=[];
kickFlag=[];


% every sub-loop randomly perturb intial condition
for kick=1:3

    % add real task pendulum integration DFC loop here
    %this add some kind of force to try and push the pendulum over as well
    x0 = [0; 1* ( rand -1); 0; 0]; 
    
    % call GetSSModel4x4V with appropriate parameters
    % run Euler integration
    [y, t, x] = SFCVLIA4x4(@CBNLVCPend, c, ssmP, ssm, K, L, t, x0);

    % get time
    newTime = (kick-1) * t(end) + t;
    
    % just show kick arrow for short time after kick
    frames = length(t);
    kickFlagK = zeros(1,frames);
    if(x0(2) > 0)
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = -abs(x0(2));
    else
        % scale arrow to size of kick
        kickFlagK(1: floor(frames/4)) = abs(x0(2));
    end

    
    % get state simulation back
    % concatenate data between runs
    tData = [tData newTime];
    xData = [xData x];
    yData = [yData y];
    kickFlag = [kickFlag kickFlagK];


   
    
end

% add plot and animation here
% plot out the state variables
 


% for all time point animate the results

range=1;

% cart not moving so set distance to zero
distance = xData(3, :);

% use animate function
step = 5;
AnimatePendulumCart( (xData(1, :)+pi),  distance, 0.64, tData, range, kickFlag, step, titleMessage);

PlotStateVariable2x2(xData, tData, titleMessage);



