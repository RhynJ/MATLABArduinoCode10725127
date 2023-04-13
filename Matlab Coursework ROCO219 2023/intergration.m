function [dx, dxHat] = intergration(ssmP,K,L,x, xhat)


A = ssmP.A; 
B = ssmP.B;
C = ssmP.C;


%before we begin we need to make sure that the system can both be controled
%and observed

%this will test the controllability of the system 
%Mc = [B AB]
ctrl = ctrb(A,B);
Mc = rank(ctrl);

%observability test
%Mo = [C; CA;]
observ = obsv(A,C);
Mo = rank(observ);

%calc the output 
%y = X(c-DK)
%there is no D matrix
y = x * C;
yHat = xhat * C;

%what we want the system to do
target = [0; 0; pi; 0]; %we want everything to tend to 0 appart from the pendulum angle

%steady state gain 
Kp = place(A, B, [-4 -5 -6 -7]);


%this is the 2 gains 
u = -K*(xHat(1:4) - target) - Kp*(x(1:4) - target); % -K*(Xe)

% using the real states (just for comparison)
dx = A*x(1:4) + B*u;

% using the estimate states
% dx = Ax + Bu + L(y - yhat)
% L(y - yhat) = L(output - output estimate)
dxhat = A*xHat(5:8) + B*u + L*(y - yHat);

dx = [dx; dxhat]; %store the new dx for both the real and simulated 
dxHat = dxhat;

end



