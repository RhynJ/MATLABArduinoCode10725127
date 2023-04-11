% calculate Ardiuino simulation of inverted pendulum
% SFC of theta, thetaDot and position
% Lueberger observer only used to estimate theta, thetaDot
% estimate position made by directly integrating the velocity control signal
% with integral action on position
% nonlinear plant simulation


close all
clear all
clc

% this will not use the default params
wantDefault = 0;

% this will get the rod params for the system
params = GetRodPendulumParams(wantDefault, 5);


% this will get the new state space coefficients
c = GetStateSpaceCoesffs(wantDefault, params);

% get state space model with thetaDot, theta
%this is the old model 

% get state space model with thetaDot, theta and  position of cart
% integral action on position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build observer just for angle and angular velocity states
% calculate observer gain L here

%this is just a check can be removed later 
ssmP = GetSSModel4x4V(params,c);




% put your code for calculating L here
PX = [-10 -11];
L = place(ssmP.AObserve, ssmP.CObserve', PX);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for state space model with thetaDot, theta and  position of cart
% calculate SFC gain K here

% put your code for calculating K here

K = place(ssmP.A,ssmP.B,[-10 -11 -12 -14]);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup time points
dt =  0.0100;
Tfinal = 5;
t = 0:dt:Tfinal;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titleMessage = 'NLSimulateVelSFCLArduino4x4 partial observer Arduino';
disp(titleMessage)

% you can base this on the code in Main_RunDemoUncontrollerPendulumV.m

%   initialize arrays
xData = [];
yData=[];
tData=[];
kickFlag=[];


% every sub-loop randomly perturb intial condition
for kick=1:3  
        
    % for each run randomly perturb intial condition
    x0 = [0; 1 * (rand - 0.5); ]; %this add some kind of force to try and push the pendulum over 
    
    % run Euler integration
    [yhat,tout,xhat] = SFCVLIA4x4(VCPendDotCB, a1, a2, b0, A, B, C, D, Ahat, K, L, t, xhat, maxSpeed);


    % get time
    newTime = (kick-1) * tout(end) + tout;
    
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
    
    % concatenate data between runs
    tData = [tData newTime];
    xData = [xData x];
    kickFlag = [kickFlag kickFlagK];
end

% add plot and animation here
% plot out the state variables
PlotStateVariable2x2(xData, tData, titleMessage);

% for all time point animate the results
figure
range=1;

% this is used to set the cart position 
distance =  size(xhat(3, :));

% use animate function
step = 5;
AnimatePendulumCart( (xData(1, :) + pi),  distance, 0.6, tData, range, kickFlag, step, titleMessage);

    

