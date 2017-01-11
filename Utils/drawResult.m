function drawResult(h, b, stDim)

L = size(b,2);

itp = linspace(1,L,100); % indexes to plot

x = b(1:stDim,:);

pointsToPlot = [x(1,:) NaN;x(2,:) NaN];

Ne = 50;% number of points in ellipse drawing
confidence = 0.9973; % draw the (3-sigma) confidence ellipse

% get covariances
for i = itp
    
    sqrtSigma = zeros(stDim,stDim);
    
    for d = 1:stDim
        sqrtSigma(:,d) = b(d*stDim+1:(d+1)*stDim, i);
    end
    
    Sigma = sqrtSigma*sqrtSigma;
    
    ptemp = error_ellipse('C',Sigma,'mu',x(:,i),'conf',confidence,'scale',0.2,'N', Ne);
    pointsToPlot = [pointsToPlot ptemp];
end      
            
set(h, 'Xdata', pointsToPlot(1,:), 'Ydata', pointsToPlot(2,:))

end