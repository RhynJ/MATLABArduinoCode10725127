function [yhat,tout,xhat] = SFCVLIA4x4(VCPendDotCB, a1, a2, b0, A, B, C, D, Ahat, Bhat, Chat, K, L, t, xhat, maxSpeed)
% nonlinear simulate state space model using C-language compatible formuation
% add real task pendulum code here


% get signal length
len = length(t) -1;

% init output
xout = zeros(2,len);

% record the initial state
xout(:, 1) = x0;
xhat = x0;
x = x0;

% luen gain
uhat = -L*xhat;

% real gain 
u = -K*x; 
      
% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:3
         
    % record time
    tout(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);
    
    % calculate state derivative from non-linear pendulum equations
    xDot = VCPendDotCB(a1, a2, b0, x, u);
    xhatDot = VCPendDotCB(a1, a2, b0, xhat, uhat);
    
    % update the state using Euler integration
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);

    %this is the simulated pen angle
    xhat(1) = xhat(1) + h * xhatDot(1);
    xhat(2) = xhat(2) + h * xhatDot(2);

    %this is used to find the cart position 
    x(3) = x(3) + h * xDot(3);
    %this will be used to find the positional error state
    x(4) = x(4) + h * xDot(4);





    %this is the output of the system
    yhat = Chat * xhat; %there is no D term do there is no need for the second part of the equastion 


    %observer correction term 
      
    % record the state
    xaproxE = x - xhat;

    %this should gove me the error calc 
    xout(:, idx) = xaproxE(A -L*C);

   


        
        
end








