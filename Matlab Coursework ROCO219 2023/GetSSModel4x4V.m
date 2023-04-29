function ssmP = GetSSModel4x4(c)
% get 4x4 state space model with thetaDot, theta and position of cart as state variables
% integral action


% add real task pendulum values here
% .....

% compute valyes
    ssmP.A = [0 1 0 0;
              -c.a2 -c.a1 0 0;
              0 0 0 0;
              0 0 1 0];   

    ssmP.B = [c.b0;
              -c.a1*c.b0; 
              1; 
              0];   

    ssmP.C = [1 0 0 0];


    ssmP.D = 0;
    
