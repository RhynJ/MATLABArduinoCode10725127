function PlotStateVariable4x4(xDirect, time, titleMessage)

% plot out the state variables
minLen = min([ length(xDirect) length(time)]);
xDirect = xDirect(:,1:minLen);
time = time(1:minLen);

% for font
params.fontSize = 20;

close all;
figure
hold on
h = plot(time, xDirect(1, :), 'b');
set(h,'LineWidth', 3);
h = plot(time, xDirect(2, :), 'r');
set(h,'LineWidth', 3);

h=legend('theta[rad]', 'x2[rad/s]');
set(h,'FontSize', params.fontSize);

% label the x-axis
h = xlabel('Time [s]');
set(h,'FontSize', params.fontSize);

% label the y-axis
h = ylabel('angle, angular velocity, displacement');
set(h,'FontSize', params.fontSize);

h = title("state varibles for 4x4");
set(h,'FontSize', params.fontSize);

% set size of axis numbering
set(gca,'FontSize', params.fontSize);

% draw y-axis zero line
h = drawXLine(0, 0, time(end), 'k:');
set(h,'LineWidth', 3);
