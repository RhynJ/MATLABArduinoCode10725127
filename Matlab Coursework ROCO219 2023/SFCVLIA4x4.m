function [yhat,tout,xhat] = SFCVLIA4x4(VCPendDotCB, a1, a2, b0, A, B, C, D, Ahat, K, L, t, xhat, maxSpeed)
% nonlinear simulate state space model using C-language compatible formuation
% add real task pendulum code here

% get signal length
len = length(t) -1;

% init output
xout = zeros(2,len); %this will set xout to 0 

% record the initial state
xout(:, 1) = x0;
xhat = x0;

% this is the control for the observer gain
u = -L*xhat;


      
% for all remaining data points, simulate state-space model using C-language compatible formulation
%this is the estimate syste
for idx = 1:len
         
    % record time
    t(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);
    
    % calculate state derivative from non-linear pendulum equations
    xDot = VCPendDotCB(a1, a2, b0, xhat, u);
    
    % update the state using Euler integration

    %these are the 2 angualr states of the cart
   
    %this is the estimate
    xhat(1) = xhat(1) + h * xDot(1);
    xhat(2) = xhat(2) + h * xDot(2);

    %this is the real system 
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);

    




    %there are no observers for these as this are not in the observer
    
    %this is used to find the cart position 
    x(3) = x(3) + h * xDot(3);
    %this will be used to find the positional error state
    x(4) = x(4) + h * xDot(4);


    % record the state
    xout(:, idx) = xhat;

    %tout(:, idx) = t;

    %this is the output of the system
    %yhat = C * xhat + D * u;

    

    

        
        
end








