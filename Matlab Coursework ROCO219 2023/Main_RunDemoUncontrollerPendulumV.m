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
% 10/02/2019
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% calculate non-linear Arduino suitable simulation of uncontrolled inverted pendulum

close all
clear all
clc

% use default parameters for demo program
wantDefault = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get parameters for rod pendulum
params = GetRodPendulumParams(wantDefault, 5);
% print out params
params

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get coefficients
c = GetStateSpaceCoesffs(wantDefault, params);
% print out values
c


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% setup time points
dt =  0.0100;
Tfinal = 5;
t = 0:dt:Tfinal;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build linearized state space matrices from differential equation
% get state space model with thetaDot, theta
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
titleMessage = 'Example Uncontrolled Inverted Pendulum: xxxYourIDHerexxx';
disp(titleMessage)

% initialize arrays
xData = [];
yData=[];
tData=[];
kickFlag=[];

% for sub-loop runs
runs = 2;
for kick=1:runs
        
    % for each run randomly perturb intial condition
    x0 = [0; 1 * (rand - 0.5); ];
    
    % run Euler integration
    [t, x] = StateSpaceIntegrator(@CBNLVCPend, c.a1, c.a2, c.b0, ssm.C, ssm.D, t, x0);
    
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


