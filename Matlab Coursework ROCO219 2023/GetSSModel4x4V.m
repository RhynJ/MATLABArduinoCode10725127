function ssmP = GetSSModel4x4(c)
% get 4x4 state space model with thetaDot, theta and position of cart as state variables
% integral action



% compute valyes
    ssmP.A = [0 1 0 0; -c.a2 -c.a1 0 0; 0 0 0 0; 0 0 1 0;]; 

    %this needs to be changed for the controlability. while it is possible
    %to calc the controlability with a 1x4 this function doesnt play well
    %with it
    ssmP.B = [c.b0 0 0 0; -c.a1*c.b0 0 0 0; 1 0 0 0; 0 1 0 0;];

    %enure that this is full rank
    ssmP.C = [0 1 0 0; 0 0 1 0; 0 0 0 1;];
    ssmP.D = 0;










    
