function xDot = CBNLPend(a1, a2, b0, x, u)
% calculate state derivative from non-inear equations
% for position controllled inverted pendulum
% x1 d/dt(pendulum angle) = pendulum angular velocity
% a1, a2, b0 are the equation coeficients
% x is the state
% u is the control input
% xDot is the returned state derivative

% compute x1Dot
xDot(1) = x(2)  + b0 * cos(x(1)) * u;

% compute x2Dot
xDot(2) = -a2 * sin(x(1)) - a1 * x(2) + (b0 * sin(x(1)) -a1 * b0 * cos(x(1))) * u;

% compute x3Dot
xDot(3) = u;

% compute x4Dot
xDot(4) = x(3); 
