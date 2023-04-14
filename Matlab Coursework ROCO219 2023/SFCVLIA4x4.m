function [tout,xout, xHatOut] = SFCVLIA4x4(K, L, t, x0, ssmP)
% nonlinear simulate state space model using C-language compatible formuation
% add real task pendulum code here


% get signal length
len = length(t) -1;

% init output
xout = zeros(4,len); % there are 4 state 
xHatOut = zeros(4,len);

% record the initial state


xout(:, 1) = x0;
xHatOut(:, 1) = x0;
xhat = x0;
x = x0;


% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:len
         
    % record time
    tout(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);
    
    % calculate state derivative from non-linear pendulum equations
    [xDot, xhatDot] = intergration(ssmP,K,L,x, xhat);

    
    % update the state using Euler integration
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);
    x(3) = x(3) + h * xDot(3);    %this is used to find the cart position
    x(4) = x(4) + h * xDot(4);    %this will be used to find the positional error state

    %this is the simulated pen 
    xhat(1) = xhat(1) + h * xhatDot(1);
    xhat(2) = xhat(2) + h * xhatDot(2);
    xhat(3) = xhat(3) + h * xhatDot(3);
    xhat(4) = xhat(4) + h * xhatDot(4);



    xout(:, idx) = x;
    xHatOut(:, idx) = xhat;

       
        
end








