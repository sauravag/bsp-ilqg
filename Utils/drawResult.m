function drawResult(plotFn, b, stDim)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw the trajectory and uncertainty ellipses
% 
% Input:
%   plotFn: function handle which sets line data
%   b: the beliefs to plot
%   stDim: robot state space dimension
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
L = size(b,2);

itp = round(linspace(1,L,50)); % indexes to plot

x = b(1:stDim,:);

pointsToPlot = [x(1,:) NaN;x(2,:) NaN];

Ne = 50;% number of points in ellipse drawing
inc= 2*pi/Ne;
phi= 0:inc:2*pi;

% get covariances
for i = itp
        
    Sigma = zeros(stDim,stDim);
    
    for d = 1:stDim
        Sigma(:,d) = b(d*stDim+1:(d+1)*stDim, i);
    end
        
    ptemp = make_ellipse(x(1:2,i),Sigma(1:2,1:2),3, phi);
    pointsToPlot = [pointsToPlot ptemp];
end      
            
plotFn(pointsToPlot);

end

function p= make_ellipse(x,P,s, phi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% make a single 2-D ellipse of s-sigmas 
% over phi angle intervals
%
% Input:
%   x: mean
%   P: sqrt of covariance
%   s: scale (1-sigma, 2-sigma etc)
%   phi: angles from o to 2*pi
%
% Output:
%   p: the points on perimiter of ellipse
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
r = sqrtm(P);
a= s*r*[cos(phi); sin(phi)];
p=[a(1,:)+x(1) NaN;a(2,:)+x(2) NaN];
end
