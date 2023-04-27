function [yhat,tout,xout] = SFCVLIA4x4(VCPendDotCB, a1, a2, b0, A, B, C, D, Ahat, K, L, t, xhat, maxSpeed)
% nonlinear simulate state space model using C-language compatible formuation
% add real task pendulum code here

% get signal length
len = length(t) -1;

% init output
xout = zeros(2,len);

% record the initial state
xout(:, 1) = x0;    %this is the input to the system
x = x0; %set x to the system input

% this is the control for the observer gain
u = -L*x;
      
% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:len
         
    % record time
    tout(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);
    
    % calculate state derivative from non-linear pendulum equations
    xDot = VCPendDotCB(a1, a2, b0, x, u);
    %this will calcualte the state derivitive for the predicted
    xhatDot = VCPendDotCB(a1, a2, b0, xhat, u); 

    % update the state using Euler integration
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);  
    x(3) = x(3) + h * xDot(3);
    x(4) = x(4) + h * xDot(4);

    xhat(1) = xhat(1) + h *xhatDot(1); %this is the observer for theta 
    xhat(2) = xhat(2) + h *xhatDot(2);  %this is the observer for thetaDot
      
    % record the state
    xout(:, idx) = x;

    
    
        
end








