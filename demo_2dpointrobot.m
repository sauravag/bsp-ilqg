%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function demo_2dpointrobot
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo for a 2D planning scenario with a point 
% robot and range sensing to beacons.
%%%%%%%%%%%%%%%%%%%%%%%%%%%

% add subfolders to path
addpath(genpath('./'));

clc;
close all;
dbstop if error;

fprintf('\n A demonstration of the iLQG algorithm for Belief Space Planning \n')

%% Initialize planning scenario

T = 50; % Total time horizon

dt = 0.2; % time step

load('Maps/mapA.mat'); % load map

mm = TwoDPointRobot(dt); % motion model

om = TwoDBeaconModel(1:size(map.landmarks,2),map.landmarks); % observation model

global ROBOT_RADIUS;
ROBOT_RADIUS = 0.46; % robot radius is needed by collision checker

svc = @(x)isStateValid(x,map); % state validity checker (collision)

% Setup start and goal/target state
x0 = map.start; % intial state
P = 0.4^2*eye(2); % intial covariance
% sqrtSigma0 = sqrtm(Sigma0);
b0 = [x0;P(:)]; % initial belief state

xf = map.goal; % target state

% 2-piece straight line guess for initial controls
u0 = repmat((xf-x0)/T,1,T/dt);% nominal controls

% Set full_DDP=true to compute 2nd order derivatives of the
% dynamics. This will make iterations more expensive, but
% final convergence will be much faster (quadratic)
full_DDP = false;

% set up the optimization problem
DYNCST  = @(b,u,i) beliefDynCost(b,u,xf,T/dt,full_DDP,mm,om,svc);

% control constraints are optional
% Op.lims  = [-2.0 2.0;         % Vx limits (m/s)
%     -2.0  2.0];        % Vy limits (m/s)

Op.plot = -1; % plot the derivatives as well

% prepare the visualization window and graphics callback
figh = figure;
drawLandmarks(figh,map.landmarks);
drawObstacles(figh,map.obstacles);
scatter(x0(1),x0(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[1.0 0.0 0.0])
scatter(xf(1),xf(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[0.0 1.0 0.0])
set(gcf,'name','Belief Space Planning with iLQG','NumberT','off');
set(gca,'Color',[0.0 0.0 0.0]);
set(gca,'xlim',map.bounds(1,[1,2]),'ylim',map.bounds(2,[1,3]),'DataAspectRatio',[1 1 1])
xlabel('X (m)'); ylabel('Y (m)');
box on

% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','r','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
Op.plotFn = plotFn;

% === run the optimization
[b,u_opt,L_opt]= iLQG(DYNCST, b0, u0, Op);

% plot the final trajectory and covariances
if animate(figh, plotFn, b0, b, u_opt, L_opt, mm, om, svc)
    warning('Robot collided during exeuction')
end

end


