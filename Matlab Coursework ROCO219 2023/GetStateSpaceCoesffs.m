function c = GetStateSpaceCoesffs(wantDefault, params)
% get state space coefficients

% if default demo program values selected
if(wantDefault)
    % dummy values set here are for demo only
    c.b0 =  3;
    c.b1 = 0;
    c.a0 = 1;
    c.a1 = 1;
    c.a2 = -25;
else
  
    % compute appropiate values here
    c.b0 = params.m*params.lh/(params.I+params.m*(params.lh^2));
    c.b1 = 0;
    c.a0 = 0;
    c.a1 = (params.mu)/(params.I+params.m*(params.lh^2));
    c.a2 =  -(params.m*params.g*params.lh)/(params.I+params.m*(params.lh^2));
end
