function  phandle = drawXLine(yPos, xMin, xMax,lineProperties)
% draw ling along y-axis at specified xPos
yy = [yPos yPos];
xx = [xMin xMax];
phandle = plot(xx, yy,  lineProperties);
