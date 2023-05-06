function [tout, xout] = StateSpaceIntegrator(VCPendDotCB, a1, a2, b0,  C, D, t, x0)
% non-linear model state space feedback velocity control using C-language compatible formulation
% performs integration by Euler's method
% VCPendDotCB is the callback function to compute xDot 
% a1, a2, b0 are coedfficients  
% C, D are the state space model output matrices
% K id the state feedback control ggain - unused here
% t is a vector of time samples
% x0 is the initial state vector
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

% get signal length
len = length(t) -1;

% init output
xout = zeros(2,len);

% record the initial state
xout(:, 1) = x0;
x = x0;

% no control
u = 0;
      
% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:len
         
    % record time
    tout(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);
    
    % calculate state derivative from non-linear pendulum equations
    xDot = VCPendDotCB(a1, a2, b0, x, u);
    
    % update the state using Euler integration
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);
      
    % record the state
    xout(:, idx) = x;

    
        
end



