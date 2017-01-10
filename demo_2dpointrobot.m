%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Belief Space Planning with iLQG
% Copyright 2017
% Author: Saurav Agarwal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function demo_2dpointrobot

% add subfolders to path
addpath(genpath('./'));

clc;
close all;
dbstop if error;

fprintf('\n A demonstration of the iLQG algorithm for Belief Space Planning \n')

% Define environment
load('Maps/mapA.mat'); % load map

% Initialize parameters
T = 60; % Total time horizon

dt = 0.2; % time step

mm = TwoDPointRobot(dt); % motion model

om = TwoDRangeModel(1:size(map.landmarks,2),map.landmarks); % observation model

cc = @(x)isStateValid(x,map); % collision checker

x0 = [-10;-10]; % intial state
Sigma0 = [1.0 0.0;0.0, 1.0]; % intial covariance
sqrtSigma0 = sqrtm(Sigma0);
b0 = [x0;sqrtSigma0(:)]; % initial belief state

xf = [0;4]; % target state

% straight line guess
u0 = [repmat(([0;-10]-x0)/(T/2),1,(T/2)/dt).*ones(mm.ctDim,(T/2)/dt),...
       repmat((xf-[0;-10])/(T/2),1,(T/2)/dt).*ones(mm.ctDim,(T/2)/dt)];% nominal controls

% Set full_DDP=true to compute 2nd order derivatives of the
% dynamics. This will make iterations more expensive, but
% final convergence will be much faster (quadratic)
full_DDP = false;

% set up the optimization problem
DYNCST  = @(b,u,i) belief_dyn_cst(b,u,xf,full_DDP,mm,om,cc);

% Op.lims  = [-10.0 10.0;         % Vx limits (m/s)
%     -10.0  10.0];        % Vy limits (m/s)

Op.plot = -1;               % plot the derivatives as well

% prepare the visualization window and graphics callback
figh = figure;
drawlandmarks(figh,map.landmarks);
drawObstacles(figh,map.obstacles);
scatter(x0(1),x0(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[1.0 0.0 0.0])
scatter(xf(1),xf(2),500,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[0.0 1.0 0.0])
set(gcf,'name','Belief Space Planning with iLQG','NumberT','off')
set(gca,'Color',[0.25 0.25 0.25]);
set(gca,'xlim',[-15 5],'ylim',[-12 5],'DataAspectRatio',[1 1 1])
xlabel('X (m)'); ylabel('Y (m)');
box on

% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','r','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));
Op.plotFn = plotFn;

% === run the optimization
[b,u,L]= iLQG(DYNCST, b0, u0, Op);

plot(b(1,:), b(2,:));

end

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = belief_dyn_cst(b,u,xf,full_DDP,motionModel,obsModel, collisionChecker)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives

beliefDim = size(b,1);
ctDim = motionModel.ctDim;

if nargout == 2
    f = beliefDynamics(b, u, motionModel, obsModel);
    c = costFunction(b, u, xf, motionModel.stDim, collisionChecker);
else
    % state and control indices
    ix = 1:beliefDim;
    iu = beliefDim+1:beliefDim+ctDim;
    
    % dynamics first derivatives
    xu_dyn  = @(xu) beliefDynamics(xu(ix,:),xu(iu,:),motionModel, obsModel);
    J       = finite_difference(xu_dyn, [b; u]);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    % dynamics second derivatives
    if full_DDP
        xu_Jcst = @(xu) finite_difference(xu_dyn, xu);
        JJ      = finite_difference(xu_Jcst, [b; u]);
        JJ      = reshape(JJ, [4 6 size(J)]);
        JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
        fxx     = JJ(:,ix,ix,:);
        fxu     = JJ(:,ix,iu,:);
        fuu     = JJ(:,iu,iu,:);
    else
        [fxx,fxu,fuu] = deal([]);
    end
    
    % cost first derivatives
    xu_cost = @(xu) costFunction(xu(ix,:),xu(iu,:),xf,motionModel.stDim, collisionChecker);
    J       = squeeze(finite_difference(xu_cost, [b; u]));
    cx      = J(ix,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    xu_Jcst = @(xu) squeeze(finite_difference(xu_cost, xu));
    JJ      = finite_difference(xu_Jcst, [b; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    cxx     = JJ(ix,ix,:);
    cxu     = JJ(ix,iu,:);
    cuu     = JJ(iu,iu,:);
    
    [f,c] = deal([]);
end
end

function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);
end

%% Drawing functions
function drawlandmarks(h, landmarks)

figure(h)
hold on;

% draw white circle around markers
for l = 1:size(landmarks,2)
%     plot(landmarks(1,l),landmarks(2,l),'.w','MarkerSize',150,'MarkerFaceAlpha',0.5);
    scatter(landmarks(1,l),landmarks(2,l),1000,'filled', ...
       'MarkerFaceAlpha',0.9,'MarkerFaceColor',[1.0 1.0 1.0])
end

plot(landmarks(1,:),landmarks(2,:),'ks',...
    'LineWidth',2,...
    'MarkerSize',8,...
    'MarkerFaceColor',[0.25,0.25,0.25])
end

function drawObstacles(h,obstacles)

figure(h)
hold on;

for i = 1:length(obstacles)
    obs = obstacles{i};
    fill(obs(1,:),obs(2,:),'m');
end

end

%% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end

function c = tt(a,b)
c = bsxfun(@times,a,b);
end