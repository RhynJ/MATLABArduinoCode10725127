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

% use default parameters for demo program this is not used
wantDefault = 0;

% get parameters for rod pendulum this get the paramter 
params = GetRodPendulumParams(wantDefault, 5);

% get state space coefficients this get the coefficents 
c = GetStateSpaceCoesffs(wantDefault, params);



% get state space model with thetaDot, theta and  position of cart

ssmP = GetSSModel4x4V(c);
ssm = GetSSModel2x2V(wantDefault, params, c);


% build observer just for angle and angular velocity states
% integral action on position the place command works out the gain for the
% system with the poles you hand it
% put your code for calculating L here
PX = [-10, -20];
L = place(ssm.A, ssm.C', PX);
disp(' ')
disp('L')
disp(L)

% for state space model with thetaDot, theta and  position of cart
% calculate SFC gain K here

 KX = [-2.1, -2.7, -3.2, - 3.8];
 K = place(ssmP.A,ssmP.B, KX);
 disp(' ')
 disp('K')
 disp(K)
 

% setup time points this is used to sample time 
dt =  0.0100;
Tfinal = 5;
t = 0:dt:Tfinal;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titleMessage = 'NLSimulateVelSFCLArduino4x4 partial observer Arduino: 10725127';
disp(titleMessage)

% you cann base thgis on the code in Main_RunDemoUncontrollerPendulumV.m
%   initialize arrays these will be used for the simulation and graph 
xData=[];
yData=[];
tData=[];
kickFlag=[];


% every sub-loop randomly perturb intial condition
for kick=1:3

    % add real task pendulum integration DFC loop here
    %this add some kind of force to try and push the pendulum over as well
    x0 = [0; 1* (rand -0.2); 0; 0]; 
    
    % call GetSSModel4x4V with appropriate parameters
    % run Euler integration this will do all of the calulation we need
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
distance = xData(3, :); %this deals with the cart position, without this the cart wont simulate

% use animate function
step = 5;
% the +pi is to start the cart position at pi in the simulation 
AnimatePendulumCart( (xData(1, :)+pi),  distance, 0.64, tData, range, kickFlag, step, titleMessage);

%this polts the system so we can analys what is happening
PlotStateVariable2x2(xData, tData, titleMessage);



