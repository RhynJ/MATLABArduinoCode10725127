function xDot = stateSpaceModel(t,x,c)
% get 2x2 state space model with thetaDot, theta as state variables


    % add real task pendulum values here
    % .....
    
    % calculate appropiate values here
    xDot.A = [0 1; -(c.a2) -(c.a1);];
    xDot.B = [c.b0; -(c.a1)*(c.b0);];
    xDot.C = [1 0;];


    %this is the gain for the real system
    xDot.K = place (A,B, [-10, -11]);   %this will work out the correct gain for me
    xDot.u = -xDot.k * x; 
    xDot = xDot.A * xDot.x + xDot.B * xDot.u;



    %this is the gain for the simulated system
    %where xhat is the estimate for the system
    xDot.L = place (A,B, [-10, -11]);   %this is the luenberger gain 
    xDot.u = -xDot.k * x; 

    xDot.hat = xDot.A * xHat + xDot.B * xDot.u; 
    
end

