function [yout, tout,xout] = SFCVLIA4x4(VCPendDotCB, c, ssmP, ssm, K, L, t, x0)
% nonlinear simulate state space model using C-language compatible formuation
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

% add real task pendulum code here
% get signal length
len = length(t) -1;

% init output
xout = zeros(4,len);
yout = zeros(4, len);
y = zeros(2, len);


% record the initial state
xout(:, 1) = x0;    %this is the input to the system

x = x0(1:4); %set x to the system input
xhat = x0(1:2); 

% this is the control for the observer gain
u = -K(1) * xhat(1) -K(2) * xhat(2) -K(3) * x(3) -K(4) * x(4);
%this is in C++ form 

% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:len
         
    % record time
    tout(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);


    % this is the control for the observer gain
    u = -K(1) * xhat(1) -K(2) * xhat(2) -K(3) * x(3) -K(4) * x(4);
    %this is in C++ form 

    
    % calculate state derivative from non-linear pendulum equations
    xDot = VCPendDotCB(c.a1, c.a2, c.b0, x, u);
    %this will calcualte the state derivitive for the predicted

    % update the state using Euler integration
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);  
    x(3) = x(3) + h * xDot(3);
    x(4) = x(4) + h * xDot(4);

    %real
    y = ssmP.C * x;
    %estimated
    yCorr = L'*(y- ssm.C * xhat);
    

    %this is the real output
    x = x + h * (ssmP.A * x + ssmP.B * u);

    %this the observer this only uses the first 2 states 
    xhatDot  = ssm.A(1:2, 1:2) * xhat + ssm.B(1:2, 1) * u + yCorr;  
    xhat = xhat + h * xhatDot; %thetahat


   
    % record the state
    xout(:, idx) = x;

    
    
        
end




