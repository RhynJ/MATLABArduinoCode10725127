function plotStateVariable4x4(xdata, xhatdata, xdotdata, xhatdotdata, thetadata,thetahatdata,thetadotdata, thetahatdotdata,timedata)

% Plot of states and state estimates

% x, xhat
figure;
subplot(2,2,1);
plot(timedata, xdata, 'b', timedata, xhatdata, 'r--');
xlabel('Time (s)');
ylabel('Cart Position (m)');
legend('Actual', 'Estimated');
title('Cart Position');

% xdot, xhatdot
subplot(2,2,2);
plot(timedata, xdotdata, 'b', timedata, xhatdotdata, 'r--');
xlabel('Time (s)');
ylabel('Cart Velocity (m/s)');
legend('Actual', 'Estimated');
title('Cart Velocity');

% theta, thetahat
subplot(2,2,3);
plot(timedata, thetadata, 'b', timedata, thetahatdata, 'r--');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('Actual', 'Estimated');
title('Pendulum Angle');

% thetadot, thetahatdot
subplot(2,2,4);
plot(timedata, thetadotdata, 'b', timedata, thetahatdotdata, 'r--');
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
legend('Actual', 'Estimated');
title('Pendulum Angular Velocity');

end


