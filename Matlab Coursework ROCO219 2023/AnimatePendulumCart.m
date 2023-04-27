function AnimatePendulumCart(thetaIn, xIn,  L, t, range, kickFlag, step, titleMessage)
% animate pendulum ads a function of angle and position
% thetaIn is pendulum angle
% xIn is pendulum position
% L is pendulum length
% t is the time array
% kickFlag is true when a kick pertubation is applied at that tmeframe
% titleMessage is the figure title
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% largely based on youTube presentation by Steve Brunton
% https://youtu.be/qjhAAQexzLg
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 23/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% check lengths
if( length(thetaIn) ~=  length(xIn))
    disp('Error in AnimatePendulumCart: length(thetaIn) ~=  length(xIn)');
    return
end
if( length(thetaIn) ~=  length(t))
    disp('Error in AnimatePendulumCart: length(thetaIn) ~=  length(t)');
    return
end
if( length(thetaIn) >  length(kickFlag))
    disp('Error in AnimatePendulumCart: length(thetaIn) > length(kickFlag)');
    return
end
if(length(L) > 1)
        disp('Error in AnimatePendulumCart: length(L) > 1)');
    return
end
if(length(range) > 1)
        disp('Error in AnimatePendulumCart: range(L) > 1)');
    return
end
if(length(titleMessage) < 1)
        disp('Error in AnimatePendulumCart: titleMessage(L) < 1)');
    return
end


% set font size
fontSize = 20;

% set arrow size
arrowSize = 40;

% width of the cart in m
W = 8/100;

% height of the cart in m
H = 8/100;

% endpoint mass radius
mr = 2/100;

% cart vertical position
y = H/2;

% location of kick arrow
kickLocation = L + mr + H/2

railDiameter=6/100;
rodDiameter=6/100;
    
% animate the results for all time point
points = length(t);
for k=2: step: points
    
    % calculate the time step
    timeStep = t(k) - t(k-1);
       
    % get position and angle values
    theta = thetaIn(k);
    x = xIn(k);
    
    % draw the rails
    h = plot([-range range],[0 0],'w');
    set(h,'LineWidth', railDiameter);
    
    % turn hold on
    hold on;
    
    % scale the arrow
    arrowSizeScaled = arrowSize * sqrt(abs(kickFlag(k)));
    
    % draw excitation arrow to represent initial kick
    if (kickFlag(k) > 0)
        % place at top of pendulum on RHS
        h = text(0, kickLocation ,'\rightarrow', 'HorizontalAlignment', 'right');
        set(h,'FontSize', arrowSizeScaled, 'Color','w');
    elseif(kickFlag(k) < 0)
        % place at top of pendulum on LHS
        h = text(0, kickLocation ,'\leftarrow', 'HorizontalAlignment', 'left');
        set(h,'FontSize', arrowSizeScaled, 'Color','w');
    end
    
    % print the frame count
    message = sprintf('Frame: %d/%d ', k, points);
    h = text(1, 0.5 ,message);
    set(h,'FontSize', fontSize, 'Color','w');
    
    % write the title
    % h = title(titleMessage);
    h = title(titleMessage);
    set(h,'FontSize', fontSize, 'Color','w');
    
    % draw the carriage
    rectangle('Position',[x-W/2,y-H/2,W,H],'Curvature',.1,'FaceColor',[1 0.1 0.1],'EdgeColor',[1 1 1]);
    
    % location of the end of the rod
    px = x + L*sin(theta);
    py = y - L*cos(theta);
    
    % draw the rod
    h=plot([x px],[y py],'w');
    set(h,'LineWidth', rodDiameter);
    
    % draw the endpoint mass
    rectangle('Position',[px-mr/2,py-mr/2,mr,mr],'Curvature',1,'FaceColor',[.3 0.3 1],'EdgeColor',[1 1 1]);
    
    % draw the pivot
    rectangle('Position',[x-mr/2,y-mr/2,mr,mr],'Curvature',1,'FaceColor',[.4 0.4 1],'EdgeColor',[1 1 1]);
    
    % set the axes range (in cm)
    xlim([-range range]);
    ylim([-range range]);
    
    %axis([-45 45 -75 75]);
    
    % maintain scaling
    axis equal;
    
    % set the colours of the plot
    set(gca,'Color','k','XColor','w','YColor','w');
    
    % set position and size of diplay window bigger than default
    set(gcf,'Position',[10 200 1000 500]);
    
    % set outside to black
    set(gcf,'Color','k');
    
    % label the x-axis
    h = xlabel('Horizontal position [m]');
    set(h,'FontSize', fontSize);
    
    % label the x-axis
    h = ylabel('Vertical position [arbirtary units]');
    set(h,'FontSize', fontSize);
    
    % set size of axis numbering
    set(gca,'FontSize', fontSize);
    
    % negative style
    set(gcf,'InvertHardcopy','off') ;
    
    % wait for time Step to give approximately real-time animation
    % ony works  time step roughly > 1/20 second
    pause(timeStep);
    
    % box off
    drawnow;
    
    % now turn off hold
    hold off;
end