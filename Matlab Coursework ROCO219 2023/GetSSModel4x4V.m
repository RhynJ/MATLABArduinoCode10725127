function ssmP = GetSSModel4x4(params, c)
% get 4x4 state space model with thetaDot, theta and position of cart as state variables
% integral action


% add real task pendulum values here
% .....

% compute valyes
    ssmP.A = [0 1 0 0; -c.a2 -c.a1 0 0; 0 0 0 0; 0 0 1 0;];   
    ssmP.B = [c.b0; -(c.a1)*(c.b0); 1; 0;];   
    ssmP.C = [1 0 0 0;];
    ssmP.D = 0;



% only observer gain
    ssmP.Ahat = [0 1; -c.a2 -c.a1;];   
    ssmP.Bhat = [c.b0; -(c.a1)*(c.b0);];   
    ssmP.Chat = [1 0;];
    ssmP.Dhat = 0;






    
