function c = costFunction(b, u, goal, stDim, collisionChecker)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for vector of states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   stDim: state space dimension for robot
%
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

L = size(b,2);

c = zeros(1,L);

for i=1:L
    c(i) =  stateCost(b(:,i),u(:,i), goal, stDim, L, collisionChecker);
end

end

function c = stateCost(b, u, goal, stDim, L, collisionChecker)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for a states according to cost model given in Section 6 
% of Van Den Berg et al. IJRR 2012
%
% Input:
%   b: Current belief vector
%   u: Control
%   goal: target state
%   stDim: State dimension
%   L: Number of steps in horizon
% Outputs:
%   c: cost estimate
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

final = isnan(u(1,:));
u(:,final)  = 0;

ctrlDim = size(u,1);

x = b(1:stDim,1);

sqrtSigma = zeros(stDim, stDim); % principal sqrt of covariance matrix

% Extract columns of principal sqrt of covariance matrix
% right now we are not exploiting symmetry
for d = 1:stDim
    sqrtSigma(:,d) = b(d*stDim+1:(d+1)*stDim, 1);
end

Q_d = 0*eye(stDim); % penalize distance to target
Q_t = 1e2*eye(stDim); % penalize uncertainty
R_t = eye(ctrlDim); % penalize control effort
Q_l = L*eye(stDim); % penalize terminal error
w_c = 1.0; % collision weight

% deviation from goal
delta_x = goal-x;

% final cost
if any(final)
  c = delta_x'*Q_l*delta_x + trace(sqrtSigma*Q_l*sqrtSigma);
else
  c = delta_x'*Q_d*delta_x + u'*R_t*u + trace(sqrtSigma*Q_t*sqrtSigma) + w_c*collisionCost(x,sqrtSigma,collisionChecker);
end

end

function f = collisionCost(x,sqrtSigma,collisionChecker)
%%%%%%%%%%%%%%%%%%%%%%%
% Compute cost for collision based on 
% Section 5 of Van den Berg et al. IJRR 2012
%
% Inputs:
%   x: robot state (belief mean)
%   sqrtSigma: principal sqrt of covariance
%   collisionChecker: function for checking collision
%%%%%%%%%%%%%%%%%%%%%%

stDim = size(x,1);

Sigma = sqrtSigma*sqrtSigma; % covariance matrix
 
eigval = eig(Sigma); % get eigen values
 
lambda = max(eigval); % get largest eigen val

d = sqrt(lambda); % distance along 1 std dev

global ROBOT_RADIUS
R_orig =  ROBOT_RADIUS; % save robot radius

% number of standard deviations at which robot collides
for s = 0:0.5:3
    % inflate robot radius 
    ROBOT_RADIUS = R_orig + s*d;
    
    % if robot collided
    if collisionChecker(x) == 0
        f = -log(chi2cdf(s^2, stDim));
        ROBOT_RADIUS = R_orig; % reset robot radius
        return;
    end
end

f = 0;
ROBOT_RADIUS = R_orig; % reset robot radius
end













