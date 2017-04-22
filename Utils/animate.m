function [failed, b_f, trCov_vs_time] = animate(figh, plotFn, b0, b_nom, u_nom, L, motionModel, obsModel, stateValidityChecker, DYNAMIC_OBS)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animate the robot's motion from start to goal
%
% Inputs:
%   figh: Figure handle in which to draw
%   plotfn: function handle to plot cov ellipse
%   b0: initial belief
%   b_nom: nominal belief trajectory
%   u_nom: nominal controls
%   L: feedback gain
%   motionModel: robot motion model
%   obsModel: observation model
% Outputs:
% failed: 0 for no collision, 1 for collision, 2 for dynamic obstacle
% detected
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stDim = motionModel.stDim;

xt = b0(1:stDim); % true state of robot
x = b0(1:stDim); % estimated mean
P = zeros(stDim); % covariance

% unpack covariance from belief vector
for d = 1:stDim
    P(:,d) = b0(d*stDim+1:(d+1)*stDim, 1);
end

rh = []; % robot disk drawing handle

figure(figh);
plot(b_nom(1,:),b_nom(2,:),'b', 'LineWidth',2);

% create robot body points
global ROBOT_RADIUS
robotDisk = ROBOT_RADIUS*[cos(linspace(0,2*pi,50));...
    sin(linspace(0,2*pi,50))];

trCov_vs_time(1) = trace(P);

roboTraj = [];

failed = 0;

for i = 1:size(u_nom,2)
    
    if DYNAMIC_OBS == 1
        if stateValidityChecker(b_nom(1:2,min(i+3,size(b_nom,2)))) == 0
            figure(figh);
            plot(roboTraj(1,:),roboTraj(2,:),'g', 'LineWidth',2);          
            drawnow;
            warning('Robot expected to Collide with dynamic obstacle!!!');
            failed = 2;
            return;
        end
    end
    
    b = [x(:);P(:)]; % current belief
    
    u = u_nom(:,i) + L(:,:,i)*(b - b_nom(:,i));
    
    % update robot
    processNoise = motionModel.generateProcessNoise(xt,u); % process noise
    xt = motionModel.evolve(xt,u,processNoise);
    
    % Get motion model jacobians and predict pose
    zeroProcessNoise = motionModel.generateProcessNoise(xt,u); % process noise
    x_prd = motionModel.evolve(x,u,processNoise); % predict robot pose
    A = motionModel.getStateTransitionJacobian(x,u,zeroProcessNoise);
    G = motionModel.getProcessNoiseJacobian(x,u,zeroProcessNoise);
    Q = motionModel.getProcessNoiseCovariance(x,u);
    
    % Get observation model jacobians
    z = obsModel.getObservation(xt); % true observation
    z_prd = obsModel.getObservation(x_prd,'nonoise'); % predicted observation
    zerObsNoise = zeros(size(z));
    H = obsModel.getObservationJacobian(x,zerObsNoise);
    M = obsModel.getObservationNoiseJacobian(x,zerObsNoise,z);
    R = obsModel.getObservationNoiseCovariance(x,z);
    
    % update P
    P_prd = A*P*A' + G*Q*G';
    S = H*P_prd*H' + M*R*M';
    K = (P_prd*H')/S;
    P = (eye(stDim) - K*H)*P_prd;
    x = x_prd + K*(z - z_prd);
    
    % final belief
    b_f = [x;P(:)];
    
    roboTraj(:,i) = x;
    
    trCov_vs_time(i+1) = trace(P);
    
    % if robot is in collision
    if stateValidityChecker(x) == 0
        figure(figh);
        plot(roboTraj(1,:),roboTraj(2,:),'g', 'LineWidth',2);
        drawnow;
        warning('Robot collided :( ');
        failed = 1;
        return;
    end

    delete(rh)
    rh = fill(x(1) + robotDisk(1,:),x(2) + robotDisk(2,:),'b');
    drawResult(plotFn,b,2);
    drawnow;
    pause(0.005);
end

figure(figh);
plot(roboTraj(1,:),roboTraj(2,:),'g', 'LineWidth',2);
drawnow;
failed = 0;
end