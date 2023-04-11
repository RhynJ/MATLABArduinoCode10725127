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



% put your code for calculating L here
PX = [-10 -11];
L = place(ssmP.AObserve, ssmP.CObserve', PX);

disp(' ')
disp('L')
disp(L)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for state space model with thetaDot, theta and  position of cart
% calculate SFC gain K here

% put your code for calculating K here

K = place(ssmP.A,ssmP.B,[-10 -11 -12 -14]);


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

% you can base this on the code in Main_RunDemoUncontrollerPendulumV.m

%   initialize arrays
xData = [];
yData=[];
tData=[];
kickFlag=[];


% every sub-loop randomly perturb intial condition
for kick=1:3 %3 might change 
        
    % for each run randomly perturb intial condition
    x0 = [0; 1 * (rand - 0.5); ]; %this add some kind of force to try and push the pendulum over 
    
    % run Euler integration
    [t, x] = SFCVLIA4x4(VCPendDotCB, a1, a2, b0, A, B, C, D, Ahat, K, L, t, xhat, maxSpeed);


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

% cart not moving so set distance to zero
distance = zeros( size(xData(1, :)));

% use animate function
step = 5;
AnimatePendulumCart( (xData(1, :) + pi),  distance, 0.6, tData, range, kickFlag, step, titleMessage);

    

