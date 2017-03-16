%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo for a 2D belief space planning scenario with a
% point robot whose body is modeled as a disk
% and it can sense beacons in the world.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plan_2dpointrobot(mapPath, outDatPath)

close all;

%% Initialize planning scenario
DYNAMIC_OBS = 1;

dt = 0.05; % time step

load(mapPath); % load map

mm = TwoDPointRobot(dt); % motion model

om = TwoDBeaconModel(1:size(map.landmarks,2),map.landmarks); % observation model

global ROBOT_RADIUS;
ROBOT_RADIUS = 0.46; % robot radius is needed by collision checker

svc = @(x)isStateValid(x,map,0); % state validity checker (collision)

%% Setup start and goal/target state

x0 = map.start; % intial state
P = 0.4^2*eye(2); % intial covariance
% sqrtSigma0 = sqrtm(Sigma0);
b0 = [x0;P(:)]; % initial belief state

xf = map.goal; % target state

%% Setup planner to get nominal controls
% planner = RRT(map,mm,svc);
planner = StraightLine(map,mm,svc);

[~,u0, initGuessFigure] = planner.plan(x0,xf);

nDT = size(u0,2); % Time steps

%% set up the optimization problem

% Set full_DDP=true to compute 2nd order derivatives of the
% dynamics. This will make iterations more expensive, but
% final convergence will be much faster (quadratic)
full_DDP = false;

% this function is needed by iLQG
DYNCST  = @(b,u,i) beliefDynCost(b,u,xf,nDT,full_DDP,mm,om,svc);

% control constraints are optional
% Op.lims  = [-2.0 2.0;         % Vx limits (m/s)
%     -2.0  2.0];        % Vy limits (m/s)

Op.plot = -1; % plot the derivatives as well

%% prepare the visualization window and graphics callback
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

%% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','r','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
Op.plotFn = plotFn;

%% === run the optimization
[b,u_opt,L_opt,~,~,optimCost,~,~,tt]= iLQG(DYNCST, b0, u0, Op);

%% Save result figure
try
    savefig(figh,strcat(outDatPath,'iLQG-solution'));
    savefig(initGuessFigure,strcat(outDatPath,'RRT-initGuess'));
catch ME
    warning('Could not save figs')
end

results.cost = fliplr(cumsum(fliplr(optimCost)));
results.b = b;
results.u = u_opt;
results.L = L_opt;
results.time = tt;
results.start = x0;
results.goal = xf;

%% plot the final trajectory and covariances
if DYNAMIC_OBS == 1
    drawObstacles(figh,map.dynamicObs);
end

svcDyn = @(x)isStateValid(x,map,DYNAMIC_OBS); % state validity checker (collision)

[didCollide, b_f] = animate(figh, plotFn, b0, b, u_opt, L_opt, mm, om, svcDyn);

results.collision = didCollide;

% if dynamic obstacle showed up
if didCollide == 2
    
    try
        savefig(figh,strcat(outDatPath,'iLQG-dynobs-collision-detected'));        
    catch ME
        warning('Could not save figs')
    end
    
    x0 = b_f(1:2,1); % intial state
    xf = map.goal; % target state
    
    planner = RRT(map,mm,svcDyn);
    
    [~,u0,~] = planner.plan(x0,xf);
    
    nDT = size(u0,2); % Time steps
    
    % this function is needed by iLQG
    DYNCST  = @(b,u,i) beliefDynCost(b,u,xf,nDT,full_DDP,mm,om,svcDyn);
    
    iLQG(DYNCST, b_f, u0, Op);
    
    try
        savefig(figh,strcat(outDatPath,'iLQG-post-dynobs-solution'));        
    catch ME
        warning('Could not save figs')
    end
end

try
    save(strcat(outDatPath,'ilqg_results.mat'), 'results');
catch ME
    warning('Could not save results')
end

end


