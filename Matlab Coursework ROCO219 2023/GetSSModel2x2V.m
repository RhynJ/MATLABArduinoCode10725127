function ssm = GetSSMode2x2V(wantDefault,  c)
% get 2x2 state space model with thetaDot, theta as state variables

% if default demo program values selected
if(wantDefault)
    
    % default values
    % for velocity control
    % % get inverted configuation
    ssm.A = [];     % we dont use A yet but you will need it later
    ssm.B = [];     % we dont use B yet but you will need it later
    ssm.C = [1 0;];
    ssm.D = 0;
    ssm.K = [];     % we dont use K yet but you will need it later
    
else
    
    % add real task pendulum values here
    % .....

    % calculate appropiate values here
    ssm.A = [0 1 0; ];   
    ssm.B = [-c.a2 -c.a1 0; 0 0 0;];   
    ssm.C = [1 0 0];
    ssm.D = 0;

end
