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
ssm = GetSSModel2x2V(wantDefault, c);

% get state space model with thetaDot, theta and  position of cart
% integral action on position
%ssmP = GetSSModel4x4V(c);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build observer just for angle and angular velocity states
% calculate observer gain L here

%this is just a check can be removed later 
ssmP = GetSSModel4x4V(c);




% put your code for calculating L here
PX = [-10 -11 -12 -13];
L = place(ssmP.A, ssmP.C', PX');


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
titleMessage = 'NLSimulateVelSFCLArduino4x4 partial observer Arduino: 10725127';
disp(titleMessage)

% you can base this on the code in Main_RunDemoUncontrollerPendulumV.m

%   initialize arrays
xData = [];
yData=[];
tData=[];
xDataEst=[];
kickFlag=[];


% every sub-loop randomly perturb intial condition
for kick=1:3  
        
    % for each run randomly perturb intial condition
    x0 = [rand * 2 - 1; rand * 2 - 1; pi; rand * 2 - 1;]; %this add some kind of force to try and push the pendulum over 

    % run Euler integration this is the simulation
    [tout,xout, xHatOut] = SFCVLIA4x4(K, L, t, x0, ssmP);


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
    xData = [xData xout];
    xDataEst = [xDataEst xHatOut];
    kickFlag = [kickFlag kickFlagK];
   
end


% for all time point animate the results
figure
range=1;

% this is used to set the cart position 
%distance =  size(xdata(3, :));

% use animate function
step = 5;
%AnimatePendulumCart( xDataEst(3, :),  xDataEst(1, :), ((params.l)), tData, range, kickFlag, step, titleMessage);


% Plotting states
plotStateVariable4x4(xData(1,:), xDataEst(1,:), xData(2,:), xDataEst(2,:), xData(3,:),xDataEst(3,:), xData(4,:), xDataEst(4,:),tData);









